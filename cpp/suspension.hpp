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

/**
 * @brief The SuspensionSystem base class, which composes the upright and hub, the suspension linkages and optionally a rocker and push/pull rod.
 *
 */
class SuspensionSystem
{
protected:
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

    /**
     * @brief Construct a new Suspension System object
     *
     * @param data
     * @param scale
     * @param forces
     * @param chassis_body
     * @param steering_body
     */
    SuspensionSystem(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &steering_body)
    {
        createUpright(data, scale, chassis_body);
        createRocker(data, scale, chassis_body);
        createMultiLink(data, scale, chassis_body, steering_body);

        createSpringDamperSystem(data, scale, forces, chassis_body);
    };

    /**
     * @brief Create a Spring Damper System object
     *
     * @param data
     * @param scale
     * @param forces
     * @param chassis_body
     */
    void createSpringDamperSystem(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body)
    {
        // TODO: The spring damper system should be able to decide where to attach to
        // or alternatively the suspension system should provide a suspension attachment frame
        m_spring_damper = SpringDamperSystem(data, scale, forces, chassis_body, m_rocker, m_upright);
    }

    /**
     * @brief Get the Upright object
     *
     * @return MobilizedBody&
     */
    MobilizedBody &getUpright()
    {
        return m_upright;
    }

    /**
     * @brief Get the Rocker object
     *
     * @return MobilizedBody&
     */
    MobilizedBody &getRocker()
    {
        return m_rocker;
    }

    /**
     * @brief Get the Hub object
     *
     * @return MobilizedBody&
     */
    MobilizedBody &getHub()
    {
        return m_hub;
    }

    /**
     * @brief Whether this Suspension System has a rocker or not
     *
     * @return true
     * @return false
     */
    bool hasRocker()
    {
        return !m_rocker.isEmptyHandle();
    }

    /**
     * @brief Create a Upright object
     *
     * @param data
     * @param scale
     * @param m_chassis
     */
    void createUpright(JSON data, Vec3 scale, MobilizedBody &m_chassis)
    {
        // TODO: Allow for compliance between spindle and upright
        // this is hopefully quite straight forward by replacing the
        // revolute joint with a linear bushing with no stiffness
        // around the wheel spin axis

        // the hub is the stationary part of the wheel bearing / spindle

        // describe mass and visualization properties for a generic body.
        auto upright_mass_props = GetMassInertia(data["suspension"]["upright"], 1.0, 1.0e-6);
        Body::Rigid uprightInfo(upright_mass_props);
        uprightInfo.addDecoration(Transform(), DecorativeSphere(0.025));

        auto hub_mass_props = GetMassInertia(data["suspension"]["hub"], 1.0, 1.0e-6);
        Body::Rigid hubInfo(hub_mass_props);
        hubInfo.addDecoration(Transform(), DecorativeCylinder(0.05, 0.10));

        Vec3 wheel_center = GetVec3(data["suspension"]["upright"]["pos"], scale);

        // upright is defined as a free mobilizer (6 DOF) w.r.t. the chassis
        m_upright = MobilizedBody::Free(m_chassis, TransformWorldToBody(m_chassis, wheel_center), uprightInfo, Transform(Vec3(0)));

        // create hub body
        m_hub = MobilizedBody::Weld(m_upright, hubInfo);
        // attach the wheel spindle mobilizer
        // m_spindle = MobilizedBody::Revolute(m_upright, TransformWorldToBody(m_upright, wheel_center, Vec3(0.0, 1.0, 0.0)), spindleInfo, Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis)));
    }

    /**
     * @brief Create a Rocker object
     *
     * @param data
     * @param scale
     * @param m_chassis
     */
    void createRocker(JSON data, Vec3 scale, MobilizedBody &m_chassis)
    {
        auto rocker_mass_props = GetMassInertia(data["suspension"]["rocker"], 1.0, 1.0e-6);
        Body::Rigid rockerInfo(rocker_mass_props);
        // rockerInfo.addDecoration(Transform(), DecorativeSphere(0.025));

        auto rocker_pos = GetVec3(data["suspension"]["rocker"]["pos"], scale);
        auto rocker_dir = GetVec3(data["suspension"]["rocker"]["dir"], scale) - rocker_pos;

        m_rocker = MobilizedBody::Pin(m_chassis, TransformWorldToBody(m_chassis, rocker_pos, rocker_dir), rockerInfo, Transform(Vec3(0)));

        auto rocker_link_pos = GetVec3(data["suspension"]["pushrod"]["rocker"], scale);
        auto upright_link_pos = GetVec3(data["suspension"]["pushrod"]["upright"], scale);

        m_pushpull = CreateLink(m_upright, upright_link_pos, m_rocker, rocker_link_pos);
    }

    /**
     * @brief Create a Multi Link object
     *
     * @param data
     * @param scale
     * @param m_chassis
     * @param m_steering
     */
    void createMultiLink(JSON data, Vec3 scale, MobilizedBody &m_chassis, MobilizedBody &m_steering)
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

        chassis_link_pos = GetVec3(data["suspension"]["lca_front"]["chassis"], scale);
        upright_link_pos = GetVec3(data["suspension"]["lca_front"]["upright"], scale);

        m_links[0] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["suspension"]["lca_rear"]["chassis"], scale);
        upright_link_pos = GetVec3(data["suspension"]["lca_rear"]["upright"], scale);

        m_links[1] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["suspension"]["uca_front"]["chassis"], scale);
        upright_link_pos = GetVec3(data["suspension"]["uca_front"]["upright"], scale);

        m_links[2] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["suspension"]["uca_rear"]["chassis"], scale);
        upright_link_pos = GetVec3(data["suspension"]["uca_rear"]["upright"], scale);

        m_links[3] = CreateLink(m_chassis, chassis_link_pos, m_upright, upright_link_pos);

        chassis_link_pos = GetVec3(data["suspension"]["toe_link"]["chassis"], scale);
        upright_link_pos = GetVec3(data["suspension"]["toe_link"]["upright"], scale);

        m_toelink = CreateLink(m_steering, chassis_link_pos, m_upright, upright_link_pos);
    }
};