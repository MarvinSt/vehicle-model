#pragma once

#include "Simbody.h"

#include "utilities/json.hpp"
#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"
#include "force_elements/ground.hpp"

#include "axle.hpp"

#include <string>
#include <fstream>

using namespace json;

using namespace SimTK;

class Chassis
{
private:
    MobilizedBody m_chassis;
    Axle m_axle[2];

public:
    Chassis() {}

    Chassis(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &ground)
    {
        CreateChassis(data, scale, ground);
        CreateAxles(data, scale, forces);
    }

    void CreateChassis(JSON data, Vec3 scale, MobilizedBody &ground, bool fixed = false)
    {
        auto constrain_chassis = false;

        // Describe mass and visualization properties for a generic body.
        Body::Rigid bodyInfo(MassProperties(200.0, Vec3(0), UnitInertia(200, 600, 600)));
        bodyInfo.addDecoration(Transform(), DecorativeSphere(0.1));

        // create dummy chassis body
        auto chassis_pos = (GetVec3(data["front"]["wheel_center"], scale) + GetVec3(data["rear"]["wheel_center"], scale)) / 2.0;
        chassis_pos = Vec3(chassis_pos[0], 0.0, 0.35);

        // ceate chassis body with 6 DOF w.r.t. the ground reference
        if (fixed)
            m_chassis = MobilizedBody::Weld(ground, Transform(chassis_pos), bodyInfo, Transform(Vec3(0)));
        else
        {
            m_chassis = MobilizedBody::Free(ground, Transform(chassis_pos), bodyInfo, Transform(Vec3(0)));

            if (constrain_chassis)
                Constraint::PointOnLine(ground, UnitVec3(0, 0, 1), chassis_pos, m_chassis, Vec3(0));
        }
    }

    void CreateAxles(JSON data, Vec3 scale, GeneralForceSubsystem &forces)
    {
        m_axle[0] = Axle(data["front"], scale, forces, m_chassis, true);
        m_axle[1] = Axle(data["rear"], scale, forces, m_chassis, false);
    }
};
