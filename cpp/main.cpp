#include "Simbody.h"
#include "json.hpp"
#include "utils.hpp"

#include "ground.hpp"

#include "springdamper.hpp"

#include <chrono>
#include <string>
#include <fstream>

using namespace std::chrono;

using namespace SimTK;
using namespace json;

struct Chassis
{
    MobilizedBody body;
};

struct Upright
{
    MobilizedBody upright;
    MobilizedBody spindle;
};

struct Geometry
{
    Constraint toelink;
    Constraint link[4];
};

struct Vehicle
{
    Chassis chassis;
    Geometry geometry[4];
    Upright upright[4];
    MobilizedBody rocker[4];
    MobilizedBody steering_rack;
    MobilizedBody steering_column;
};

struct Suspension
{
    MobilizedBody rocker;
};

MobilizedBody CreateSteeringSystem(JSON hardpoints, MobilizedBody &chassis_body)
{
    /*
        The steering system consists of:
        - steering rack (slider): lateral w.r.t. chassis
        - pinion (1d velocity constraint): translation to rotational w.r.t. rack
        - steering shaft (revolute): fixed to pinion and rotates w.r.t. chassis
        - intermediate shaft (universal): with respect to steering shaft
        - steering column (universal):

        This part is not really ideal in terms of computational efficiency.
        It probaby makes sense to model up to the rack mounted revolute joint and
        lump the rest of the steering system inertia/mass alltogether in a single shaft.
    */

    auto steering_simplified = true;

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    // Take the tierod inner location as the end point of the steering rack
    // with steering rack position in the middle of the chassis
    auto steering_rack_end = GetVec3(hardpoints["tierod_inner"], scale);
    auto steering_rack_pos = steering_rack_end.elementwiseMultiply(Vec3(1.0, 0.0, 1.0));

    // Calculate steering rack direction and length
    auto steering_rack_dir = (steering_rack_end - steering_rack_pos);
    auto steering_rack_len = steering_rack_dir.norm() * 2.0;

    // Describe mass and visualization properties for a generic body.
    Body::Rigid steeringRackInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
    steeringRackInfo.addDecoration(Transform(FromDirectionVector(Vec3(1.0, 0, 0), YAxis)), DecorativeCylinder(0.01, steering_rack_len / 2.0));

    // A slider is defined along the common X-axis, redirect it along the steering axis
    // auto steering_rack = MobilizedBody::Slider(chassis_body, Transform(FromDirectionVector(steering_rack_dir, XAxis), PosWorldToBody(chassis_body, steering_rack_pos)), steeringRackInfo, Transform(Vec3(0)));
    auto steering_rack = MobilizedBody::Slider(chassis_body, TransformWorldToBody(chassis_body, steering_rack_pos, steering_rack_dir, XAxis), steeringRackInfo, Transform(Vec3(0)));

    // Get steering shaft and column coordinates
    auto pinion_center_at_rack = GetVec3(hardpoints["pinion_center_at_rack"], scale);
    auto intermediate_shaft_forward = GetVec3(hardpoints["intermediate_shaft_forward"], scale);
    auto intermediate_shaft_rear = GetVec3(hardpoints["intermediate_shaft_rear"], scale);
    auto steeringwheel_center = GetVec3(hardpoints["steeringwheel_center"], scale);

    auto seering_column_dir = steeringwheel_center - intermediate_shaft_rear;

    // Calculate virtual contact point to achieve desired steering ration
    auto steering_rack_ratio = 0.0024; // [rev/mm]
    auto effective_pinion_radius = (1.0 / steering_rack_ratio) * 1.0 / (2.0 * Pi) / 1000.0;

    // Steering shaft and column mobilizers
    // All joints are defined at the base for convenience (expressed in world frame coordinates converted to body local)

    // Steering shaft
    auto steering_shaft_dir = intermediate_shaft_forward - pinion_center_at_rack;
    auto steering_shaft_len = steering_shaft_dir.norm();

    Body::Rigid steeringShaftInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
    steeringShaftInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, steering_shaft_len / 2.0));

    auto steering_shaft = MobilizedBody::Revolute(chassis_body, TransformWorldToBody(chassis_body, pinion_center_at_rack, steering_shaft_dir), steeringShaftInfo, Transform(Vec3(0.0, 0.0, -steering_shaft_len / 2.0)));

    // Project a point at the center of the steering rack onto the steering shaft to find the virtual perpendicular contact point
    auto pinion_virtual_cen = ProjectPointOnLine(steering_rack_pos, pinion_center_at_rack, intermediate_shaft_forward);
    auto pinion_virtual_contact_dir = steering_rack_pos - pinion_virtual_cen;
    // auto pinion_virtual_contact_pos = UnitVec3(pinion_virtual_contact_dir) * effective_pinion_radius + pinion_virtual_cen;
    auto pinion_virtual_contact_pos = UnitVec3(pinion_virtual_contact_dir) * effective_pinion_radius + pinion_center_at_rack;

    // Add a no slip 1d constraints to constrain the rotation, we need to use the virtual contact position here to get the correct steering ratio
    Constraint::NoSlip1D(chassis_body, PosWorldToBody(chassis_body, pinion_virtual_contact_pos), UnitVec3(steering_rack_dir), steering_rack, steering_shaft);

    if (steering_simplified)
    {
        // Induce artificial motion
        Motion::Sinusoid(steering_shaft, Motion::Level::Position, 20.0 * Pi / 180.0, 2.0 * 2.0 * Pi, 0.0);

        return steering_rack;
    }

    // Intermediate shaft
    auto intermediate_shaft_dir = intermediate_shaft_rear - intermediate_shaft_forward;
    auto intermediate_shaft_len = intermediate_shaft_dir.norm();

    Body::Rigid intermediateShaftInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
    intermediateShaftInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, intermediate_shaft_len / 2.0));

    auto intermediate_shaft = MobilizedBody::Universal(steering_shaft, TransformWorldToBody(steering_shaft, intermediate_shaft_forward, intermediate_shaft_dir), intermediateShaftInfo, Transform(Vec3(0.0, 0.0, -intermediate_shaft_len / 2.0)));

    // Steering column
    auto steering_columm_dir = steeringwheel_center - intermediate_shaft_rear;
    auto steering_column_len = steering_columm_dir.norm();

    // Split the body mass and inertia in half, because we need both a chassis mounted and intermediate shaft mounted part which are welded together
    Body::Rigid steeringColumnInfo(MassProperties(1.0 / 2.0, Vec3(0), UnitInertia(0.01) / 2.0));
    auto steering_column = MobilizedBody::Universal(intermediate_shaft, TransformWorldToBody(intermediate_shaft, intermediate_shaft_rear, steering_columm_dir), steeringColumnInfo, Transform(Vec3(0.0, 0.0, -steering_column_len / 2.0)));
    steeringColumnInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, steering_column_len / 2.0));
    steeringColumnInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0, 0.0, steering_column_len / 2.0)), DecorativeBrick(Vec3(0.125, 0.025, 0.05)));
    auto steering_column_cha = MobilizedBody::Revolute(chassis_body, TransformWorldToBody(chassis_body, intermediate_shaft_rear, steering_columm_dir, ZAxis), steeringColumnInfo, Transform(Vec3(0.0, 0.0, -steering_column_len / 2.0)));
    // auto steering_column_cha = MobilizedBody::Revolute(steering_column, TransformWorldToBody(steering_column, intermediate_shaft_rear, steering_columm_dir, ZAxis), steeringColumnInfo, Transform(Vec3(0.0, 0.0, -steering_column_len / 2.0)));

    // Weld revolute chassis mounted joint to steering column
    Constraint::Weld(steering_column_cha, steering_column);

    // Induce artificial motion
    Motion::Sinusoid(steering_shaft, Motion::Level::Position, 20.0 * Pi / 180.0, 2.0 * 2.0 * Pi, 0.0);

    return steering_rack;
}

void CreateAntiRollbar(JSON hardpoints, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &rocker_body_left, MobilizedBody &rocker_body_right)
{
    // Define arb stiffness
    auto torsion_stiffness = 1.00 * 1000.0 / 180.0 * Pi; // base unit N*mm/deg
    auto torsion_initial_angle = 0.0;

    // Describe mass and visualization properties for a generic body.
    Body::Rigid arbInfo(MassProperties(0.1, Vec3(0), UnitInertia(0.1)));

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    auto arb_bend = GetVec3(hardpoints["arb_bend"], scale);
    auto arb_middle = Vec3(arb_bend[0], 0.0, arb_bend[2]);

    auto arb_droplink = GetVec3(hardpoints["droplink_to_arb"], scale);
    auto droplink_rocker = GetVec3(hardpoints["arblink_to_bellcrank"], scale);
    auto rocker_position = GetVec3(hardpoints["bellcrank_pivot"], scale);

    auto arb_rot_dir = arb_bend - arb_middle;

    auto arb_len = arb_rot_dir.norm() * 2.0;
    arbInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), XAxis)), DecorativeCylinder(0.01, arb_len / 2.0));

    // Attach a revolute mobilizer to the chassis, this will assure a rotational degree of freedom of the ARB w.r.t. the chassis
    auto arb_body_left = MobilizedBody::Revolute(chassis_body, TransformWorldToBody(chassis_body, arb_middle, arb_rot_dir), arbInfo, Transform());

    // Attach the right hand side body (revolute w.r.t. right hand side body)
    auto arb_body_right = MobilizedBody::Revolute(arb_body_left, TransformWorldToBody(arb_body_left, arb_middle, arb_rot_dir), arbInfo, Transform());
    // auto arb_body_right = MobilizedBody::Revolute(chassis_body, TransformWorldToBody(chassis_body, arb_middle, arb_rot_dir), arbInfo, Transform());

    // Define torsionbar stiffness for the anti-rollbar
    Force::MobilityLinearSpring(forces, arb_body_right, MobilizerUIndex(0), torsion_stiffness, torsion_initial_angle);

    // Attach the droplink from the rocker body onto to the arb body
    CreateLink(arb_body_left, arb_droplink, rocker_body_left, droplink_rocker);
    CreateLink(arb_body_right, arb_droplink.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)), rocker_body_right, droplink_rocker.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)));

    // Induce artificial motion
    // Motion::Sinusoid(arb_body_left, Motion::Level::Position, 10.0 * Pi / 180.0, 4.0 * 2.0 * Pi, 0.0);
}

void CreateSpringDampersInboard(JSON hardpoints, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &rocker_body, bool left = true)
{
    auto stiffness = 60.0 * 1000.0; // spring stiffness
    auto damping = 1.0 * 1000.0;    // damping ratio
    auto x0 = 0.0;                  // compression at rest length

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    if (!left)
        scale = FlipAxis(scale, 1);

    auto chassis_link_pos = GetVec3(hardpoints["shock_to_chassis"], scale);
    auto rocker_link_pos = GetVec3(hardpoints["shock_to_bellcrank"], scale);

    auto dist = (chassis_link_pos - rocker_link_pos).norm();

    // add the spring rest length
    x0 += dist;

    // define tabular spring damper element
    ForceElement::TabularSpringDamper *tabular_spring = new ForceElement::TabularSpringDamper(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), x0);

    tabular_spring->push_spring_table(-1.0, -stiffness);
    tabular_spring->push_spring_table(+1.0, +stiffness);

    tabular_spring->push_damper_table(-1.0, -damping);
    tabular_spring->push_damper_table(+1.0, +damping);

    const ForceElement::TabularSpringDamper &tabular_spring_ref = *tabular_spring;
    Force::Custom(forces, tabular_spring);

    /*
    // for debugging purposes
    std::cout << "force val +1.0: " << tabular_spring->eval_force(x0 + 1.0) << std::endl;
    std::cout << "force val  0.0: " << tabular_spring->eval_force(x0 + 0.0) << std::endl;
    std::cout << "force val -1.0: " << tabular_spring->eval_force(x0 - 1.0) << std::endl;

    std::cout << "tab size: " << tabular_spring->get_spring_table_size() << std::endl;
    */

    // Standard linear spring implementation
    // Force::TwoPointLinearSpring(forces, chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), stiffness, x0);
    // Force::TwoPointLinearDamper(forces, chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), damping);
}

void CreateSpringDampersOutboard(JSON hardpoints, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &upright_body, bool left = true)
{
    auto stiffness = 100.0; // spring stiffness
    auto damping = 1.0;     // damping ratio
    auto x0 = 0.0;          // compression at rest length

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    if (!left)
        scale = FlipAxis(scale, 1);

    auto chassis_link_pos = GetVec3(hardpoints["prod_to_bellcrank"], scale);
    auto upright_link_pos = GetVec3(hardpoints["prod_outer"], scale);

    auto dist = (chassis_link_pos - upright_link_pos).norm();

    // add the spring rest length
    x0 += dist;

    Force::TwoPointLinearSpring(forces, chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), upright_body, PosWorldToBody(upright_body, upright_link_pos), stiffness, x0);
    Force::TwoPointLinearDamper(forces, chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), upright_body, PosWorldToBody(upright_body, upright_link_pos), damping);
}

Upright CreateUpright(JSON hardpoints, MobilizedBody &chassis_body, bool left = true)
{
    Upright upright;

    // Describe mass and visualization properties for a generic body.
    Body::Rigid uprightInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.1)));
    uprightInfo.addDecoration(Transform(), DecorativeSphere(0.025));

    Body::Rigid wheelInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.1)));
    wheelInfo.addDecoration(Transform(), DecorativeCylinder(0.25, 0.10));

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    if (!left)
        scale = FlipAxis(scale, 1);

    Vec3 wheel_center = GetVec3(hardpoints["wheel_center"], scale);

    upright.upright = MobilizedBody::Free(chassis_body, TransformWorldToBody(chassis_body, wheel_center), uprightInfo, Transform(Vec3(0)));

    // Attach the wheel mobilizer
    // TODO: Include static toe and camber here (probably we should just orient the upright accordingly...)
    // upright.spindle = MobilizedBody::Revolute(upright.upright, Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis), PosWorldToBody(upright.upright, wheel_center)), wheelInfo, Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis)));
    upright.spindle = MobilizedBody::Revolute(upright.upright, TransformWorldToBody(upright.upright, wheel_center, Vec3(0.0, 1.0, 0.0)), wheelInfo, Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis)));

    Body::Rigid driveShaftInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.1)));
    driveShaftInfo.addDecoration(Transform(), DecorativeCylinder(0.01, 0.5));

    // Create driveshafts
    // MobilizedBody::Gimbal(upright.spindle, TransformWorldToBody(upright.spindle, wheel_center, Vec3(0.0, 1.0, 0.0)), driveShaftInfo, Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), ZAxis), Vec3(0.0, -0.5, 0.0)));

    // Constraint::PointOnLine()

    Motion::Steady(upright.spindle, 10.0);

    return upright;
}

Geometry CreateMultiLink(JSON hardpoints, MobilizedBody &chassis_body, MobilizedBody &steering_body, MobilizedBody &upright_body, bool left = true)
{
    Vec3 chassis_link_pos;
    Vec3 upright_link_pos;

    Geometry geom;

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    if (!left)
        scale = FlipAxis(scale, 1);

    chassis_link_pos = GetVec3(hardpoints["lca_front"], scale);
    upright_link_pos = GetVec3(hardpoints["lca_outer"], scale);

    geom.link[0] = CreateLink(chassis_body, chassis_link_pos, upright_body, upright_link_pos);

    chassis_link_pos = GetVec3(hardpoints["lca_rear"], scale);
    upright_link_pos = GetVec3(hardpoints["lca_outer"], scale);

    geom.link[1] = CreateLink(chassis_body, chassis_link_pos, upright_body, upright_link_pos);

    chassis_link_pos = GetVec3(hardpoints["uca_front"], scale);
    upright_link_pos = GetVec3(hardpoints["uca_outer"], scale);

    geom.link[2] = CreateLink(chassis_body, chassis_link_pos, upright_body, upright_link_pos);

    chassis_link_pos = GetVec3(hardpoints["uca_rear"], scale);
    upright_link_pos = GetVec3(hardpoints["uca_outer"], scale);

    geom.link[3] = CreateLink(chassis_body, chassis_link_pos, upright_body, upright_link_pos);

    chassis_link_pos = GetVec3(hardpoints["tierod_inner"], scale);
    upright_link_pos = GetVec3(hardpoints["tierod_outer"], scale);

    // correction for steering frame
    geom.toelink = CreateLink(steering_body, chassis_link_pos, upright_body, upright_link_pos);

    return geom;
}

MobilizedBody CreateRocker(JSON hardpoints, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &upright_body, bool left = true)
{
    Body::Rigid rockerInfo(MassProperties(0.1, Vec3(0), UnitInertia(0.001)));
    // rockerInfo.addDecoration(Transform(), DecorativeSphere(0.025));

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    if (!left)
        scale = FlipAxis(scale, 1);

    auto rocker_pos = GetVec3(hardpoints["bellcrank_pivot"], scale);
    auto rocker_dir = GetVec3(hardpoints["bellcrank_pivot_orient"], scale) - rocker_pos;

    auto transform_rocker_body = TransformWorldToBody(chassis_body, rocker_pos, rocker_dir);

    auto rocker = MobilizedBody::Pin(chassis_body, transform_rocker_body, rockerInfo, Transform(Vec3(0)));

    auto rocker_link_pos = GetVec3(hardpoints["prod_to_bellcrank"], scale);
    auto upright_link_pos = GetVec3(hardpoints["prod_outer"], scale);

    auto dist = (rocker_link_pos - upright_link_pos).norm();

    CreateLink(upright_body, upright_link_pos, rocker, rocker_link_pos);

    return rocker;
}

void CreateWheelContact(GeneralForceSubsystem &forces, MobilizedBody &upright, Real unloaded_radius)
{
    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    // if (!left)
    //     scale = FlipAxis(scale, 1);

    // auto rocker_pos = GetVec3(hardpoints["bellcrank_pivot"], scale);

    const Real k = 100 * 1000; // vertical tire stiffness
    const Real c = 500;        // vertical damping

    Force::Custom(forces, new WheelContact(UnitVec3(0.0, 0.0, 1.0), upright, Vec3(0.0, 0.0, -unloaded_radius), k, c));
}

int main()
{
    auto step_size = 1.0e-3;

    bool skip_viz = false;
    bool constrain_chassis = false;
    bool test_model = false;

    auto t_end = 20.0;

    // Define the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -ZAxis, 9.81);

    // Describe mass and visualization properties for a generic body.
    Body::Rigid bodyInfo(MassProperties(200.0, Vec3(0), UnitInertia(200, 600, 600)));
    bodyInfo.addDecoration(Transform(), DecorativeSphere(0.1));

    // Load parameters
    auto data = LoadParameters("../parameters/kinematics.json");

    Vehicle vehicle;

    // Create chassis body
    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;
    auto chassis_pos = (GetVec3(data["front"]["wheel_center"], scale) + GetVec3(data["rear"]["wheel_center"], scale)) / 2.0;
    chassis_pos = Vec3(chassis_pos[0], 0.0, 0.35);

    auto ground = matter.Ground();

    vehicle.chassis.body = MobilizedBody::Free(ground, Transform(chassis_pos), bodyInfo, Transform(Vec3(0)));

    if (constrain_chassis)
        Constraint::PointOnLine(ground, UnitVec3(0, 0, 1), chassis_pos, vehicle.chassis.body, Vec3(0));

    for (int i = 0; i < 4; i++)
    {
        bool left = i == 0 || i == 2;
        JSON hardpoints = i < 2 ? data["front"] : data["rear"];

        if (i == 0)
        {
            // Generate steering rack
            vehicle.steering_rack = CreateSteeringSystem(hardpoints, vehicle.chassis.body);
        }

        vehicle.upright[i] = CreateUpright(hardpoints, vehicle.chassis.body, left);
        if (i < 2)
            vehicle.geometry[i] = CreateMultiLink(hardpoints, vehicle.chassis.body, vehicle.steering_rack, vehicle.upright[i].upright, left);
        else
            vehicle.geometry[i] = CreateMultiLink(hardpoints, vehicle.chassis.body, vehicle.chassis.body, vehicle.upright[i].upright, left);

        vehicle.rocker[i] = CreateRocker(hardpoints, forces, vehicle.chassis.body, vehicle.upright[i].upright, left);
        CreateSpringDampersInboard(hardpoints, forces, vehicle.chassis.body, vehicle.rocker[i], left);

        if (!left)
        {
            // Attach ARB assemblies
            CreateAntiRollbar(hardpoints, forces, vehicle.chassis.body, vehicle.rocker[i - 1], vehicle.rocker[i - 0]);
        }

        CreateWheelContact(forces, vehicle.upright[i].upright, 0.20);
    }

    // Set up visualization.
    if (!skip_viz)
    {
        system.setUseUniformBackground(true);
        Visualizer viz(system);
        system.addEventReporter(new Visualizer::Reporter(viz, 0.01));
    }

    // Initialize the system and state.
    State state = system.realizeTopology();
    // pendulum2.setRate(state, 5.0);

    // Simulate for 20 seconds.
    // RungeKuttaMersonIntegrator integ(system);
    // RungeKutta3Integrator integ(system);
    SemiExplicitEulerIntegrator integ(system, step_size);

    // integ.setFixedStepSize(step_size);
    TimeStepper ts(system, integ);
    ts.initialize(state);

    if (test_model)
        t_end = 0.01;

    auto start = high_resolution_clock::now();
    ts.stepTo(t_end);
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    std::cout << "Time taken by function: "
              << duration.count() / 1000.0 << " [ms] " << t_end / (duration.count() / 1.0e6) << " [RTF] " << std::endl;
}