#pragma once

#include "Simbody.h"

#include "utilities/json.hpp"
#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"

#include <string>
#include <fstream>

using namespace json;

using namespace SimTK;

class AntiRollbar
{
private:
    MobilizedBody m_arb_left;
    MobilizedBody m_arb_right;

    Force m_torsional_stiffess;

public:
    AntiRollbar() {}

    AntiRollbar(JSON data, Vec3 scale, MobilizedBody &chassis_body, MobilizedBody arb_attachment_body[2])
    {
        // create anti-rollbar bodies and kinematic links only, without torsional stiffness
        CreateAntiRollbar(data, scale, chassis_body, arb_attachment_body[0], arb_attachment_body[1]);
    }

    AntiRollbar(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody arb_attachment_body[2])
    {
        CreateAntiRollbar(data, scale, chassis_body, arb_attachment_body[0], arb_attachment_body[1]);
        CreateTorsionalStiffness(forces);
    }

    void CreateAntiRollbar(JSON data, Vec3 scale, MobilizedBody &chassis_body, MobilizedBody &rocker_body_left, MobilizedBody &rocker_body_right)
    {
        auto arb_bend = GetVec3(data["arb_bend"], scale);
        auto arb_middle = Vec3(arb_bend[0], 0.0, arb_bend[2]);

        auto arb_droplink = GetVec3(data["droplink_to_arb"], scale);
        auto droplink_rocker = GetVec3(data["arblink_to_bellcrank"], scale);

        auto arb_rot_dir = arb_bend - arb_middle;
        auto arb_len = arb_rot_dir.norm() * 2.0;

        // Describe mass and visualization properties for a generic body.
        Body::Rigid arbInfo(MassProperties(0.1, Vec3(0), UnitInertia(0.1)));
        arbInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), XAxis)), DecorativeCylinder(0.01, arb_len / 2.0));

        // Attach a revolute mobilizer to the chassis, this will assure a rotational degree of freedom of the ARB w.r.t. the chassis
        m_arb_left = MobilizedBody::Revolute(chassis_body, TransformWorldToBody(chassis_body, arb_middle, arb_rot_dir), arbInfo, Transform());

        // Attach the right hand side body (revolute w.r.t. right hand side body)
        m_arb_right = MobilizedBody::Revolute(m_arb_left, TransformWorldToBody(m_arb_left, arb_middle, arb_rot_dir), arbInfo, Transform());

        // Attach the droplink from the rocker body onto to the arb body
        CreateLink(m_arb_left, arb_droplink, rocker_body_left, droplink_rocker);
        CreateLink(m_arb_right, arb_droplink.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)), rocker_body_right, droplink_rocker.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)));
    }

    void CreateTorsionalStiffness(GeneralForceSubsystem &forces)
    {
        // Define arb stiffness
        auto torsion_stiffness = 1.00 * 1000.0 / 180.0 * Pi; // base unit N*mm/deg
        auto torsion_initial_angle = 0.0;

        // Define torsionbar stiffness for the anti-rollbar
        m_torsional_stiffess = Force::MobilityLinearSpring(forces, m_arb_right, MobilizerUIndex(0), torsion_stiffness, torsion_initial_angle);
    }
};