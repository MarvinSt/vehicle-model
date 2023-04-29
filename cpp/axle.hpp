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
/**
 * @brief The Axle base class, which composes left and right side suspension, tires and optional anti-roll bar and steering system
 *
 */
class Axle
{
protected:
    Wheel m_wheel[2];
    SuspensionSystem m_suspension[2];
    SteeringSystem m_steering;
    AntiRollbar m_arb;

public:
    Axle() {}

    /**
     * @brief Construct a new Axle object
     *
     * @param data
     * @param scale
     * @param forces
     * @param chassis_body
     * @param steering
     */
    Axle(JSON data, Vec3 scale, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, bool steering = true)
    {
        // create steering system (this can probably be done implicitly by checking the JSON data)
        if (steering)
            m_steering = SteeringSystem(data, scale, chassis_body);

        // steering body for the toe link connection (steering rack if available, otherwise chassis)
        auto steering_body = m_steering.hasSteering() ? m_steering.getRack() : chassis_body;

        // create kinematics
        m_suspension[0] = SuspensionSystem(data, scale, forces, chassis_body, steering_body);
        m_suspension[1] = SuspensionSystem(data, FlipAxis(scale, 1), forces, chassis_body, steering_body);

        // create anti-rollbar
        // TODO: this should be done differently, the ARB should be able to decide the attachment body
        // to do this properly, it should take a reference to the kinematics directly
        MobilizedBody arb_attachment_bodies[] = {m_suspension[0].getRocker(), m_suspension[1].getRocker()};
        m_arb = AntiRollbar(data, scale, forces, chassis_body, arb_attachment_bodies);

        // create wheels
        m_wheel[0] = Wheel(m_suspension[0].getHub(), forces, 0.20);
        m_wheel[1] = Wheel(m_suspension[1].getHub(), forces, 0.20);
    }

    /**
     * @brief Get the Anti Rollbar Frame object
     *
     * @param side
     * @param rocker_mounted
     * @return MobilizedBody&
     */
    MobilizedBody &getAntiRollbarFrame(int side, bool rocker_mounted = false)
    {
        return rocker_mounted ? getRocker(side) : getUpright(side);
    }

    /**
     * @brief Get the Steering object
     *
     * @return SteeringSystem&
     */
    SteeringSystem &getSteering()
    {
        return m_steering;
    }

    /**
     * @brief Get the Rocker object
     *
     * @param side
     * @return MobilizedBody&
     */
    MobilizedBody &getRocker(int side)
    {
        return m_suspension[side].getRocker();
    }

    /**
     * @brief Get the Upright object
     *
     * @param side
     * @return MobilizedBody&
     */
    MobilizedBody &getUpright(int side)
    {
        return m_suspension[side].getUpright();
    }

    /**
     * @brief Get the Wheel object
     *
     * @param side
     * @return MobilizedBody&
     */
    MobilizedBody &getWheel(int side)
    {
        return m_wheel[side].getWheel();
    }
};
