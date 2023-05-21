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
    MobilizedBody m_driveshaft[2];

public:
    Axle() {}

    MobilizedBody BuildTripodJointMassless(MobilizedBody &root, Vec3 tripod_pos, Vec3 driveshaft_pos)
    {
        /* WORK IN PROGRESS... */

        // get root position
        auto root_pos = PosBodyToWorld(root, Vec3(0));

        // get direction vectors
        auto d_PA = UnitVec3(tripod_pos - root_pos);
        auto d_PB = UnitVec3(tripod_pos - driveshaft_pos);

        auto ang_z = acos(dot(d_PA, -d_PB));
        auto rev_rot_z = Transform(Rotation().setRotationFromAngleAboutZ(ang_z / 2.0));

        // calculate joint orientations
        // direct the y-joint along the bisector, this causes the x-join axes to coincide
        auto rev_dir_x = cross(d_PA, d_PB);
        auto rev_dir_y = UnitVec3(d_PA + d_PB);

        // dummy body
        Body::Rigid emptyBodyInfo(MassProperties(0, Vec3(0), Inertia(Vec3(0))));

        // Rev Joint 1 : X --> input joint
        auto transform_1 = TransformWorldToBody(root, tripod_pos, rev_dir_x);
        auto rev_x_1 = MobilizedBody::Revolute(root, transform_1, emptyBodyInfo, Transform());

        // Rev Joint 2 : Y --> mid joint
        auto transform_2 = TransformWorldToBody(rev_x_1, tripod_pos, rev_dir_y);
        auto rev_y_2 = MobilizedBody::Revolute(rev_x_1, transform_2, emptyBodyInfo, Transform());

        // Rev Joint 3 : X --> output joint
        auto transform_3 = TransformWorldToBody(rev_y_2, tripod_pos, rev_dir_x);
        auto rev_x_3 = MobilizedBody::Revolute(rev_y_2, transform_3, emptyBodyInfo, Transform());

        return rev_x_3;
    }

    MobilizedBody BuildTripodJoint(MobilizedBody &root, Vec3 tripod_pos, Vec3 driveshaft_pos)
    {
        /* WORK IN PROGRESS... */

        /*
        |  rev joint
        |  -->  o---------   <-- output shaft
        |      /
        |     /   <-- input shaft
        |    /
        |   /
        */

        // get root position
        auto root_pos = PosBodyToWorld(root, Vec3(0));

        // get direction vectors
        auto d_PA = UnitVec3(tripod_pos - root_pos);
        auto d_PB = UnitVec3(tripod_pos - driveshaft_pos);

        // frame rotation
        auto ang_z = acos(dot(d_PA, -d_PB));
        auto rev_rot_z = Transform(Rotation().setRotationFromAngleAboutZ(-ang_z / 2.0));
        auto direction = root_pos[1] < 0.0 ? 1.0 : -1.0;

        // calculate joint orientations
        // direct the y-joint along the bisector, this causes the x-join axes to coincide
        auto rev_dir_x = cross(d_PA, d_PB);
        auto rev_dir_y = UnitVec3(d_PA + d_PB);

        // dummy body
        Body::Rigid emptyBodyInfo(MassProperties(0, Vec3(0), Inertia(Vec3(0))));

        // actual driveshaft body
        auto driveshaft_len = 2.0 * (tripod_pos - driveshaft_pos).norm();
        auto driveshaft_mass = MassProperties(0.1, Vec3(0), Inertia(Vec3(0.001, 0.001, 0.001)));
        Body::Rigid driveShaftBodyInfo(driveshaft_mass);
        driveShaftBodyInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), XAxis)), DecorativeCylinder(0.01, driveshaft_len / 2.0));

        // Rev Joint 1 : X --> input joint
        auto transform_1 = TransformWorldToBody(root, tripod_pos, rev_dir_x) * rev_rot_z;
        auto rev_x_1 = MobilizedBody::Revolute(root, transform_1, emptyBodyInfo, Transform());

        // Rev Joint 2 : Y --> mid joint
        auto transform_2 = TransformWorldToBody(rev_x_1, tripod_pos, rev_dir_y);
        auto rev_y_2 = MobilizedBody::Revolute(rev_x_1, transform_2, emptyBodyInfo, Transform());

        // Rev Joint 3 : X --> output joint
        // auto transform_3 = TransformWorldToBody(rev_y_2, tripod_pos, rev_dir_x);
        // auto rev_x_3 = MobilizedBody::Revolute(rev_y_2, transform_3, emptyBodyInfo, Transform());
        auto reorient = FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis).invert();

        auto transform_3 = TransformWorldToBody(rev_y_2, tripod_pos, rev_dir_x) * rev_rot_z * rev_rot_z;
        auto rev_x_3 = MobilizedBody::Revolute(rev_y_2, transform_3, driveShaftBodyInfo, Transform(reorient, Vec3(0.0, 0.0, direction * driveshaft_len / 2.0)));

        return rev_x_3;
    }

    int BuildDriveshaft(MobilizedBody &chassis_body, MobilizedBody &wheel, Vec3 tripod_pos_whl, Vec3 tripod_pos_cha, int side)
    {
        /* WORK IN PROGRESS... */
        // compute driveshaft direction and length
        auto driveshaft_pos = (tripod_pos_cha + tripod_pos_whl) / 2.0;
        auto driveshaft_dir = tripod_pos_cha - tripod_pos_whl;
        auto driveshaft_len = driveshaft_dir.norm();

        auto diff_pos_cha = tripod_pos_cha.elementwiseMultiply(Vec3(1.0, 0.0, 1.0));

        // define driveshaft body (should be devided by half, since we need to weld two driveshaft halves together)
        auto driveshaft_mass = MassProperties(0.1, Vec3(0), Inertia(Vec3(0.001, 0.001, 0.001)));
        Body::Rigid driveShaftBodyInfo(driveshaft_mass);
        driveShaftBodyInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), XAxis)), DecorativeCylinder(0.01, driveshaft_len / 2.0));

        // create wheel side tripod joint and weld driveshaft body

        auto tripod_joint_whl = BuildTripodJointMassless(wheel, tripod_pos_whl, driveshaft_pos);
        auto driveshaft_whl = MobilizedBody::Weld(tripod_joint_whl,
                                                  TransformWorldToBody(tripod_joint_whl, driveshaft_pos, driveshaft_dir),
                                                  driveShaftBodyInfo, Transform());

        /*
        auto driveshaft_whl = BuildTripodJoint(wheel, tripod_pos_whl, driveshaft_pos);
        */
        // create differential output body
        auto diff_output_cha = MobilizedBody::Cylinder(chassis_body,
                                                       TransformWorldToBody(chassis_body, diff_pos_cha, tripod_pos_cha - diff_pos_cha, ZAxis),
                                                       driveShaftBodyInfo, Transform());

        // create chassis side tripod joint

        auto tripod_joint_cha = BuildTripodJointMassless(diff_output_cha, tripod_pos_cha, driveshaft_pos);

        // weld dummy driveshaft
        auto driveshaft_cha = MobilizedBody::Weld(tripod_joint_cha,
                                                  TransformWorldToBody(tripod_joint_cha, driveshaft_pos, driveshaft_pos - tripod_pos_cha),
                                                  driveShaftBodyInfo, Transform());
        /*
        auto driveshaft_cha = BuildTripodJoint(diff_output_cha, tripod_pos_cha, driveshaft_pos);
        */

        // weld the dummy driveshaft to the actual body
        Constraint::Weld(driveshaft_whl, driveshaft_cha);

        // differential output, all we need here is a primatic joint to allow sliding of the tripd
        m_driveshaft[side] = diff_output_cha;

        return 0;
    }

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
        m_wheel[0] = Wheel(data, m_suspension[0].getHub(), forces, 0.20);
        m_wheel[1] = Wheel(data, m_suspension[1].getHub(), forces, 0.20);

        // driveshaft // WORK IN PROGRESS
        return;

        if (!data.hasKey("driveshaft"))
            return;

        auto wheel = m_wheel[0].getWheel();
        auto driveshaft_offset = data["driveshaft"]["offset"].ToFloat();
        auto driveshaft_pos = GetVec3(data["driveshaft"]["chassis"], scale) + Vec3(-0.2, 0.0, -0.2) * 1.0;
        auto tripod_pos = PosBodyToWorld(wheel, Vec3(0));
        tripod_pos[1] = tripod_pos[1] - sign(tripod_pos[1]) * driveshaft_offset * 1.0e-3;
        BuildDriveshaft(chassis_body, wheel, tripod_pos, driveshaft_pos, 0);

        wheel = m_wheel[1].getWheel();
        BuildDriveshaft(chassis_body, wheel, FlipAxis(tripod_pos, 1), FlipAxis(driveshaft_pos, 1), 1);
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

    MobilizedBody &getDriveShaft(int side)
    {
        return m_driveshaft[side];
    }
};
