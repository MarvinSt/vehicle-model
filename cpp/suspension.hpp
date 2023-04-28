#pragma once

#include "Simbody.h"

#include "spring_damper.hpp"

#include "utilities/json.hpp"
#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"

#include <string>
#include <fstream>

using namespace json;

using namespace SimTK;

class SuspensionSystem
{
private:
    // locally created bodies
    MobilizedBody m_hub;
    MobilizedBody m_upright;
    MobilizedBody m_rocker;

    // suspension link variant specific properties
    Constraint m_links[4];
    Constraint m_toelink;
    Constraint m_pushpull;

    // spring damper system handle
    SpringDamperSystem m_spring_damper;

public:
    SuspensionSystem(){};

    SuspensionSystem(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &steering_body)
    {
        CreateUpright(data, scale, chassis_body);
        CreateRocker(data, scale, chassis_body);
        CreateMultiLink(data, scale, chassis_body, steering_body);

        CreateSpringDamperSystem(data, scale, forces, chassis_body);
    };

    void CreateSpringDamperSystem(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body)
    {
        // TODO: The spring damper system should be able to decide where to attach to
        // or alternatively the suspension system should provide a suspension attachment frame
        m_spring_damper = SpringDamperSystem(data, scale, forces, chassis_body, m_rocker, m_upright);
    }

    MobilizedBody &GetUpright()
    {
        return m_upright;
    }

    MobilizedBody &GetRocker()
    {
        return m_rocker;
    }

    MobilizedBody &GetHub()
    {
        return m_hub;
    }

    bool HasRocker()
    {
        return !m_rocker.isEmptyHandle();
    }

    void CreateUpright(JSON data, Vec3 scale, MobilizedBody &m_chassis)
    {
        // TODO: Allow for compliance between spindle and upright
        // this is hopefully quite straight forward by replacing the
        // revolute joint with a linear bushing with no stiffness
        // around the wheel spin axis

        // the hub is the stationary part of the wheel bearing / spindle

        // describe mass and visualization properties for a generic body.
        Body::Rigid uprightInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.1)));
        uprightInfo.addDecoration(Transform(), DecorativeSphere(0.025));

        Body::Rigid hubInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.1)));
        hubInfo.addDecoration(Transform(), DecorativeCylinder(0.05, 0.10));

        Vec3 wheel_center = GetVec3(data["wheel_center"], scale);

        // upright is defined as a free mobilizer (6 DOF) w.r.t. the chassis
        m_upright = MobilizedBody::Free(m_chassis, TransformWorldToBody(m_chassis, wheel_center), uprightInfo, Transform(Vec3(0)));

        // create hub body
        m_hub = MobilizedBody::Weld(m_upright, hubInfo);
        // attach the wheel spindle mobilizer
        // m_spindle = MobilizedBody::Revolute(m_upright, TransformWorldToBody(m_upright, wheel_center, Vec3(0.0, 1.0, 0.0)), spindleInfo, Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis)));
    }

    void CreateRocker(JSON data, Vec3 scale, MobilizedBody &m_chassis)
    {
        Body::Rigid rockerInfo(MassProperties(0.1, Vec3(0), UnitInertia(0.001)));
        // rockerInfo.addDecoration(Transform(), DecorativeSphere(0.025));

        auto rocker_pos = GetVec3(data["bellcrank_pivot"], scale);
        auto rocker_dir = GetVec3(data["bellcrank_pivot_orient"], scale) - rocker_pos;

        m_rocker = MobilizedBody::Pin(m_chassis, TransformWorldToBody(m_chassis, rocker_pos, rocker_dir), rockerInfo, Transform(Vec3(0)));

        auto rocker_link_pos = GetVec3(data["prod_to_bellcrank"], scale);
        auto upright_link_pos = GetVec3(data["prod_outer"], scale);

        m_pushpull = CreateLink(m_upright, upright_link_pos, m_rocker, rocker_link_pos);
    }

    void CreateMultiLink(JSON data, Vec3 scale, MobilizedBody &m_chassis, MobilizedBody &m_steering)
    {
        /*
        |   ^ y+
        |   |             wheel center
        |   +--> x+           ^
        |                     |
        | uca_outer ->     O--O--O      <- lca_outer
        |                 / \   / \
        | upper     ->   /   \ /   \    <- lower control arm
        | control arm   /     \     \
        |             ===   / ===    \
        |              ^  ===        ===
        |              |              ^
        |           uca_rear          |
        |                         lca_front
        */

        Vec3 chassis_link_pos;
        Vec3 upright_link_pos;

        chassis_link_pos = GetVec3(data["lca_front"], scale);
        upright_link_pos = GetVec3(data["lca_outer"], scale);

        m_links[0] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["lca_rear"], scale);
        upright_link_pos = GetVec3(data["lca_outer"], scale);

        m_links[1] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["uca_front"], scale);
        upright_link_pos = GetVec3(data["uca_outer"], scale);

        m_links[2] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["uca_rear"], scale);
        upright_link_pos = GetVec3(data["uca_outer"], scale);

        m_links[3] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["tierod_inner"], scale);
        upright_link_pos = GetVec3(data["tierod_outer"], scale);

        m_toelink = CreateLink(m_steering, chassis_link_pos, m_upright, upright_link_pos);
    }
};