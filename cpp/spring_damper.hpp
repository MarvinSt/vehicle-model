#pragma once

#include "Simbody.h"

#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"

using namespace json;

using namespace SimTK;

class SpringDamperSystem
{
private:
    ForceElement::TabularSpringDamper *tabular_spring;

public:
    SpringDamperSystem(){};

    SpringDamperSystem(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &rocker_body, MobilizedBody &upright_body)
    {
        CreateSpringDampersInboard(data, scale, forces, chassis_body, rocker_body);
        CreateSpringDampersOutboard(data, scale, forces, chassis_body, upright_body);
    }

    void CreateSpringDampersInboard(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &rocker_body)
    {
        auto stiffness = 60.0 * 1000.0; // spring stiffness
        auto damping = 1.0 * 1000.0;    // damping ratio
        auto x0 = 0.0;                  // compression at rest length

        auto chassis_link_pos = GetVec3(data["shock_to_chassis"], scale);
        auto rocker_link_pos = GetVec3(data["shock_to_bellcrank"], scale);

        auto dist = (chassis_link_pos - rocker_link_pos).norm();

        // add the spring rest length
        x0 += dist;

        // define tabular spring damper element
        tabular_spring = new ForceElement::TabularSpringDamper(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), x0);

        // push linear data (for now)
        tabular_spring->push_spring_table(-1.0, -stiffness);
        tabular_spring->push_spring_table(+1.0, +stiffness);

        tabular_spring->push_damper_table(-1.0, -damping);
        tabular_spring->push_damper_table(+1.0, +damping);

        const ForceElement::TabularSpringDamper &tabular_spring_ref = *tabular_spring;
        Force::Custom(forces, tabular_spring);
    }

    void CreateSpringDampersOutboard(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &upright_body)
    {
        // not implemented... but the idea is the same
    }
};
