#pragma once

#include "Simbody.h"

#include "utilities/json.hpp"
#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"

#include <string>
#include <fstream>

using namespace json;

using namespace SimTK;

/**
 * @brief The AntiRollbar base class composing the left/right side anti-rollbar bodies joined by the torsional spring stiffness.
 *
 */
class AntiRollbar
{
protected:
    MobilizedBody m_arb_left;
    MobilizedBody m_arb_right;

    Force m_torsional_stiffess;

public:
    AntiRollbar() {}

    /**
     * @brief Construct a new Anti Rollbar object (kinematics only)
     *
     * @param data
     * @param scale
     * @param chassis_body
     * @param arb_attachment_body
     */
    AntiRollbar(JSON data, Vec3 scale, MobilizedBody &chassis_body, MobilizedBody arb_attachment_body[2])
    {
        // create anti-rollbar bodies and kinematic links only, without torsional stiffness
        createAntiRollbar(data, scale, chassis_body, arb_attachment_body[0], arb_attachment_body[1]);
    }

    /**
     * @brief Construct a new Anti Rollbar object (including torsional stiffness)
     *
     * @param data
     * @param scale
     * @param forces
     * @param chassis_body
     * @param arb_attachment_body
     */
    AntiRollbar(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody arb_attachment_body[2])
    {
        createAntiRollbar(data, scale, chassis_body, arb_attachment_body[0], arb_attachment_body[1]);
        createTorsionalStiffness(forces);
    }

    /**
     * @brief Create a Anti Rollbar object
     *
     * @param data
     * @param scale
     * @param chassis_body
     * @param rocker_body_left
     * @param rocker_body_right
     */
    void createAntiRollbar(JSON data, Vec3 scale, MobilizedBody &chassis_body, MobilizedBody &rocker_body_left, MobilizedBody &rocker_body_right)
    {
        auto arb_bend = GetVec3(data["antirollbar"]["arb"]["bend"], scale);
        auto arb_middle = GetVec3(data["antirollbar"]["arb"]["center"], scale);

        auto arb_droplink = GetVec3(data["antirollbar"]["droplink"]["arb"], scale);
        auto droplink_rocker = GetVec3(data["antirollbar"]["droplink"]["rocker"], scale);

        auto arb_rot_dir = arb_bend - arb_middle;
        auto arb_len = arb_rot_dir.norm() * 2.0;

        // Describe mass and visualization properties for a generic body.
        auto arb_mass_props = GetMassInertia(data["antirollbar"]["arb"], 1.0, 1.0e-6);
        Body::Rigid arbInfo(arb_mass_props);
        arbInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), XAxis)), DecorativeCylinder(0.01, arb_len / 2.0));

        // Attach a revolute mobilizer to the chassis, this will assure a rotational degree of freedom of the ARB w.r.t. the chassis
        m_arb_left = MobilizedBody::Revolute(chassis_body, TransformWorldToBody(chassis_body, arb_middle, arb_rot_dir), arbInfo, Transform());

        // Attach the right hand side body (revolute w.r.t. right hand side body)
        m_arb_right = MobilizedBody::Revolute(m_arb_left, TransformWorldToBody(m_arb_left, arb_middle, arb_rot_dir), arbInfo, Transform());

        // Attach the droplink from the rocker body onto to the arb body
        CreateLink(m_arb_left, arb_droplink, rocker_body_left, droplink_rocker);
        CreateLink(m_arb_right, arb_droplink.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)), rocker_body_right, droplink_rocker.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)));
    }

    /**
     * @brief Create a Torsional Stiffness object
     *
     * @param forces
     */
    void createTorsionalStiffness(GeneralForceSubsystem &forces)
    {
        // Define arb stiffness
        auto torsion_stiffness = 1.00 * 1000.0 / 180.0 * Pi; // base unit N*mm/deg
        auto torsion_initial_angle = 0.0;

        // Define torsionbar stiffness for the anti-rollbar
        m_torsional_stiffess = Force::MobilityLinearSpring(forces, m_arb_right, MobilizerUIndex(0), torsion_stiffness, torsion_initial_angle);
    }
};