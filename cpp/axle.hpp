#pragma once

#include "Simbody.h"

#include "anti_rollbar.hpp"
#include "steering.hpp"
#include "suspension.hpp"
#include "wheel.hpp"

#include "utilities/json.hpp"
#include "utilities/utils.hpp"
#include "force_elements/spring_damper.hpp"

using namespace json;

using namespace SimTK;

class Axle
{
private:
    Wheel m_wheel[2];
    SuspensionSystem m_suspension[2];
    SteeringSystem m_steering;
    AntiRollbar m_arb;

public:
    Axle() {}

    Axle(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, bool steering = true)
    {
        // create steering system (this can probably be done implicitly by checking the JSON data)
        if (steering)
            m_steering = SteeringSystem(data, scale, chassis_body);

        // steering body for the toe link connection (steering rack if available, otherwise chassis)
        auto steering_body = m_steering.HasSteering() ? m_steering.GetRack() : chassis_body;

        // create kinematics
        m_suspension[0] = SuspensionSystem(data, scale, forces, chassis_body, steering_body);
        m_suspension[1] = SuspensionSystem(data, FlipAxis(scale, 1), forces, chassis_body, steering_body);

        // create anti-rollbar
        // TODO: this should be done differently, the ARB should be able to decide the attachment body
        // to do this properly, it should take a reference to the kinematics directly
        MobilizedBody arb_attachment_bodies[] = {m_suspension[0].GetRocker(), m_suspension[1].GetRocker()};
        m_arb = AntiRollbar(data, scale, forces, chassis_body, arb_attachment_bodies);

        // create wheels
        m_wheel[0] = Wheel(m_suspension[0].GetHub(), forces, 0.20);
        m_wheel[1] = Wheel(m_suspension[1].GetHub(), forces, 0.20);
    }

    MobilizedBody &GetAntiRollbarFrame(int side, bool rocker_mounted = false)
    {
        return rocker_mounted ? GetRocker(side) : GetUpright(side);
    }

    SteeringSystem &GetSteering()
    {
        return m_steering;
    }

    MobilizedBody &GetRocker(int side)
    {
        return m_suspension[side].GetRocker();
    }

    MobilizedBody &GetUpright(int side)
    {
        return m_suspension[side].GetUpright();
    }

    MobilizedBody GetWheel(int side)
    {
        return m_wheel[side].GetWheel();
    }
};
