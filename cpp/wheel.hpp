#pragma once

#include "Simbody.h"

#include "utilities/utils.hpp"
#include "utilities/json.hpp"
#include "force_elements/ground.hpp"

using namespace json;

using namespace SimTK;

/**
 * @brief The Wheel base class, which composes the WheelContact collider and wheel body
 *
 */
class Wheel
{
protected:
    MobilizedBody m_wheel;
    // Tire m_tire;
    // Terrain m_terrain (could actually be part of the tire?)

    WheelContact *m_contact;

public:
    Wheel() {}

    Wheel(MobilizedBody &hub_body) {}

    /**
     * @brief Construct a new Wheel object
     *
     * @param hub_body
     * @param forces
     * @param r_unloaded
     */
    Wheel(MobilizedBody &hub_body, GeneralForceSubsystem &forces, Real r_unloaded)
    {
        createWheel(hub_body);
        createWheelCollider(hub_body, forces, r_unloaded);
    }

    /**
     * @brief Create a Wheel object
     *
     * @param hub_body
     */
    void createWheel(MobilizedBody &hub_body)
    {
        Body::Rigid wheelInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.1)));
        wheelInfo.addDecoration(Transform(), DecorativeCylinder(0.25, 0.10));

        // attach the wheel spindle mobilizer
        auto hub_center = PosBodyToWorld(hub_body, Vec3(0));
        m_wheel = MobilizedBody::Revolute(hub_body, TransformWorldToBody(hub_body, hub_center, Vec3(0.0, 1.0, 0.0)), wheelInfo, Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis)));
    }

    /**
     * @brief Create a Wheel Collider object
     *
     * @param hub_body
     * @param forces
     * @param r_unloaded
     */
    void createWheelCollider(MobilizedBody &hub_body, GeneralForceSubsystem &forces, Real r_unloaded)
    {
        const Real k = 100 * 1000; // vertical tire stiffness
        const Real c = 500;        // vertical damping

        // Attach a wheel contact collider to the hub, this is important, because we need to project straight
        // down along the upright and the spindle is rotating. We can't use the upright body because of possible hub compliance
        m_contact = new WheelContact(UnitVec3(0.0, 0.0, 1.0), hub_body, Vec3(0.0, 0.0, -r_unloaded), k, c);
        Force::Custom(forces, m_contact);
    }

    /**
     * @brief Get the Wheel object
     *
     * @return MobilizedBody&
     */
    MobilizedBody &getWheel()
    {
        return m_wheel;
    }
};
