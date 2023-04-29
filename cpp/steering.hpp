#pragma once

#include "Simbody.h"

#include "chassis.hpp"

#include "utilities/json.hpp"
#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"

#include <string>
#include <fstream>

using namespace json;

using namespace SimTK;

/**
 * @brief The SteeringSystem base class composing the various steering system bodies (including steering rack and shaft).
 *
 */
class SteeringSystem
{
protected:
    MobilizedBody m_steering_rack;
    MobilizedBody m_steering_shaft;

public:
    SteeringSystem() {}

    /**
     * @brief Construct a new Steering System object
     *
     * @param data dictionary with the steering system hardpoints and inertia data
     * @param scale scaling applied to the hardpoints, to allow for unit conversion and mirroring
     * @param chassis_body the steering system is mounted to this body
     */
    SteeringSystem(JSON data, Vec3 scale, MobilizedBody &chassis_body)
    {
        createSteeringSystem(data, scale, chassis_body);
    }

    /**
     * @brief Get the Rack object
     *
     * @return MobilizedBody&
     */
    MobilizedBody &getRack()
    {
        return m_steering_rack;
    }

    /**
     * @brief Get the Steering Shaft object
     *
     * @return MobilizedBody&
     */
    MobilizedBody &getSteeringShaft()
    {
        return m_steering_shaft;
    }

    /**
     * @brief The steering system exists or not
     *
     * @return true
     * @return false
     */
    bool hasSteering()
    {
        return !m_steering_rack.isEmptyHandle();
    }

    /**
     * @brief Create a Steering System object
     *
     * @param data dictionary with the steering system hardpoints and inertia data
     * @param scale scaling applied to the hardpoints, to allow for unit conversion and mirroring
     * @param m_chassis the steering system is mounted to this body
     * @param simplified whether to simplify the steering system or not (neglect beyond steering shaft)
     */
    void createSteeringSystem(JSON data, Vec3 scale, MobilizedBody &m_chassis, bool simplified = true)
    {
        /*
            Steering System Overview:

            | steering column --------->     |
            |                         O------|  <-- steering wheel
            | intermediate shaft --> /       |
            |                       O
            |                  \   /  <-- steering shaft
            | steering pinion   \ /
            |                    O
            |                     \
            | steering rack   -->  \
        */

        /*
            The steering system consists of:
            - steering rack (slider): lateral w.r.t. chassis
            - pinion (1d velocity constraint): translation to rotational w.r.t. rack
            - steering shaft (revolute): fixed to pinion and rotates w.r.t. chassis
            - intermediate shaft (universal): with respect to steering shaft
            - steering column (universal):

            This part is not really ideal in terms of computational efficiency.
            It probaby makes sense to model up to the rack mounted revolute joint and
            lump the rest of the steering system inertia/mass alltogether in a single shaft.
        */

        // Take the tierod inner location as the end point of the steering rack
        // with steering rack position in the middle of the chassis
        auto steering_rack_end = GetVec3(data["tierod_inner"], scale);
        auto steering_rack_pos = steering_rack_end.elementwiseMultiply(Vec3(1.0, 0.0, 1.0));

        // Calculate steering rack direction and length
        auto steering_rack_dir = (steering_rack_end - steering_rack_pos);
        auto steering_rack_len = steering_rack_dir.norm() * 2.0;

        // Describe mass and visualization properties for a generic body.
        Body::Rigid steeringRackInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
        steeringRackInfo.addDecoration(Transform(FromDirectionVector(Vec3(1.0, 0, 0), YAxis)), DecorativeCylinder(0.01, steering_rack_len / 2.0));

        // A slider is defined along the common X-axis, redirect it along the steering axis
        m_steering_rack = MobilizedBody::Slider(m_chassis, TransformWorldToBody(m_chassis, steering_rack_pos, steering_rack_dir, XAxis), steeringRackInfo, Transform(Vec3(0)));

        // Get steering shaft and column coordinates
        auto pinion_center_at_rack = GetVec3(data["pinion_center_at_rack"], scale);
        auto intermediate_shaft_forward = GetVec3(data["intermediate_shaft_forward"], scale);
        auto intermediate_shaft_rear = GetVec3(data["intermediate_shaft_rear"], scale);
        auto steeringwheel_center = GetVec3(data["steeringwheel_center"], scale);

        auto seering_column_dir = steeringwheel_center - intermediate_shaft_rear;

        // Calculate virtual contact point to achieve desired steering ration
        auto steering_rack_ratio = 0.0024; // [rev/mm]
        auto effective_pinion_radius = (1.0 / steering_rack_ratio) * 1.0 / (2.0 * Pi) / 1000.0;

        // Steering shaft and column mobilizers
        // All joints are defined at the base for convenience (expressed in world frame coordinates converted to body local)

        // Steering shaft
        auto steering_shaft_dir = intermediate_shaft_forward - pinion_center_at_rack;
        auto steering_shaft_len = steering_shaft_dir.norm();

        Body::Rigid steeringShaftInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
        steeringShaftInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, steering_shaft_len / 2.0));

        m_steering_shaft = MobilizedBody::Revolute(m_chassis, TransformWorldToBody(m_chassis, pinion_center_at_rack, steering_shaft_dir), steeringShaftInfo, Transform(Vec3(0.0, 0.0, -steering_shaft_len / 2.0)));

        // Project a point at the center of the steering rack onto the steering shaft to find the virtual perpendicular contact point
        auto pinion_virtual_cen = ProjectPointOnLine(steering_rack_pos, pinion_center_at_rack, intermediate_shaft_forward);
        auto pinion_virtual_contact_dir = steering_rack_pos - pinion_virtual_cen;
        // auto pinion_virtual_contact_pos = UnitVec3(pinion_virtual_contact_dir) * effective_pinion_radius + pinion_virtual_cen;
        auto pinion_virtual_contact_pos = UnitVec3(pinion_virtual_contact_dir) * effective_pinion_radius + pinion_center_at_rack;

        // Add a no slip 1d constraints to constrain the rotation, we need to use the virtual contact position here to get the correct steering ratio
        Constraint::NoSlip1D(m_chassis, PosWorldToBody(m_chassis, pinion_virtual_contact_pos), UnitVec3(steering_rack_dir), m_steering_rack, m_steering_shaft);

        // Induce artificial motion
        Motion::Sinusoid(m_steering_shaft, Motion::Level::Position, 20.0 * Pi / 180.0, 2.0 * 2.0 * Pi, 0.0);

        if (simplified)
        {
            // nothing further...
            return;
        }

        // Intermediate shaft
        auto intermediate_shaft_dir = intermediate_shaft_rear - intermediate_shaft_forward;
        auto intermediate_shaft_len = intermediate_shaft_dir.norm();

        Body::Rigid intermediateShaftInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
        intermediateShaftInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, intermediate_shaft_len / 2.0));

        auto intermediate_shaft = MobilizedBody::Universal(m_steering_shaft, TransformWorldToBody(m_steering_shaft, intermediate_shaft_forward, intermediate_shaft_dir), intermediateShaftInfo, Transform(Vec3(0.0, 0.0, -intermediate_shaft_len / 2.0)));

        // Steering column
        auto steering_columm_dir = steeringwheel_center - intermediate_shaft_rear;
        auto steering_column_len = steering_columm_dir.norm();

        // Split the body mass and inertia in half, because we need both a chassis mounted and intermediate shaft mounted part which are welded together
        Body::Rigid steeringColumnInfo(MassProperties(1.0 / 2.0, Vec3(0), UnitInertia(0.01) / 2.0));
        auto steering_column = MobilizedBody::Universal(intermediate_shaft, TransformWorldToBody(intermediate_shaft, intermediate_shaft_rear, steering_columm_dir), steeringColumnInfo, Transform(Vec3(0.0, 0.0, -steering_column_len / 2.0)));
        steeringColumnInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, steering_column_len / 2.0));
        steeringColumnInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0, 0.0, steering_column_len / 2.0)), DecorativeBrick(Vec3(0.125, 0.025, 0.05)));
        auto steering_column_cha = MobilizedBody::Revolute(m_chassis, TransformWorldToBody(m_chassis, intermediate_shaft_rear, steering_columm_dir, ZAxis), steeringColumnInfo, Transform(Vec3(0.0, 0.0, -steering_column_len / 2.0)));

        // Weld revolute chassis mounted joint to steering column
        Constraint::Weld(steering_column_cha, steering_column);
    }
};
