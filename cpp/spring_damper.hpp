#pragma once

#include "Simbody.h"

#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"

#include <memory>

using namespace json;

using namespace SimTK;

/**
 * @brief The SpringDamperSystem base class composing the TabularSpringDamper system suspending the upright (either directly or indirectly, i.e. rocker-mounted).
 *
 */
class SpringDamperSystem
{
protected:
    // std::unique_ptr<ForceElement::TabularSpringDamper> tabular_spring;
    ForceElement::TabularSpringDamper *tabular_spring;

public:
    SpringDamperSystem(){};

    /**
     * @brief Construct a new Spring Damper System object
     *
     * @param data dictionary with the mounting hardpoints and suspension data
     * @param scale scaling applied to the hard points (allowing for unit conversion and mirroring)
     * @param forces the general forces subsystem of the multibody solver
     * @param chassis_body used for attaching base (stationary) side of the spring damper system
     * @param rocker_body used for attaching end (moving) side of the rocker mounted spring damper system
     * @param upright_body used for attaching the (moving) side of the upright mounted spring damper system
     */
    SpringDamperSystem(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &rocker_body, MobilizedBody &upright_body)
    {
        createSpringDampersInboard(data, scale, forces, chassis_body, rocker_body);
        createSpringDampersOutboard(data, scale, forces, chassis_body, upright_body);
    }

    /**
     * @brief Create a Spring Dampers Inboard object (rocker mounted)
     *
     * @param data dictionary with the mounting hardpoints and suspension data
     * @param scale scaling applied to the hard points (allowing for unit conversion and mirroring)
     * @param forces the general forces subsystem of the multibody solver
     * @param chassis_body used for attaching base (stationary) side of the spring damper system
     * @param rocker_body used for attaching end (moving) side of the rocker mounted spring damper system
     */
    void createSpringDampersInboard(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &rocker_body)
    {
        auto stiffness = 60.0 * 1000.0; // spring stiffness
        auto damping = 1.0 * 1000.0;    // damping ratio
        auto x0 = 0.0;                  // compression at rest length

        auto chassis_link_pos = GetVec3(data["suspension"]["spring_damper"]["chassis"], scale);
        auto rocker_link_pos = GetVec3(data["suspension"]["spring_damper"]["rocker"], scale);

        auto dist = (chassis_link_pos - rocker_link_pos).norm();

        // add the spring rest length
        x0 += dist;

        // define tabular spring damper element
        // tabular_spring = std::unique_ptr<ForceElement::TabularSpringDamper>(new ForceElement::TabularSpringDamper(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), x0));
        tabular_spring = new ForceElement::TabularSpringDamper(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), x0);

        // push linear data (for now)
        tabular_spring->push_spring_table(-1.0, -stiffness);
        tabular_spring->push_spring_table(+1.0, +stiffness);

        tabular_spring->push_damper_table(-1.0, -damping);
        tabular_spring->push_damper_table(+1.0, +damping);

        // const ForceElement::TabularSpringDamper &tabular_spring_ref = *tabular_spring;
        // Force::Custom(forces, tabular_spring.get());
        Force::Custom(forces, tabular_spring);
    }

    /**
     * @brief Create a Spring Dampers Outboard object (upright mounted)
     *
     * @param data dictionary with the mounting hardpoints and suspension data
     * @param scale scaling applied to the hard points (allowing for unit conversion and mirroring)
     * @param forces the general forces subsystem of the multibody solver
     * @param chassis_body used for attaching base (stationary) side of the spring damper system
     * @param upright_body used for attaching the (moving) side of the upright mounted spring damper system
     */
    void createSpringDampersOutboard(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &upright_body)
    {
        // not implemented... but the idea is the same
    }
};
