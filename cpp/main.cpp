#include "Simbody.h"
#include "json.hpp"

#include <string>
#include <fstream>

#include <chrono>

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
};

struct Suspension
{
    MobilizedBody rocker;
};

Vec3 GetVec3(JSON data, Vec3 scale = Vec3(1.0, 1.0, 1.0))
{
    return Vec3(data[0].ToFloat(), data[1].ToFloat(), data[2].ToFloat()).elementwiseMultiply(scale);
}

Vec3 FlipAxis(Vec3 in, int axis)
{
    in[axis] = -in[axis];
    return in;
}

Rotation RotationFromDirection(Vec3 direction = Vec3(0.0, 0.0, 1.0), CoordinateAxis axis = ZAxis)
{
    return Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);
}

Transform GetGlobalTransform(MobilizedBody body)
{
    auto transform = Transform(Vec3(0));
    while (true)
    {
        transform = (body.getDefaultInboardFrame()) * transform;
        auto parent_body = body.getParentMobilizedBody();

        if (parent_body.isSameMobilizedBody(body) || parent_body.isGround())
            break;

        body = parent_body;
    }
    return transform;
}

Vec3 PosWorldToBody(MobilizedBody body, Vec3 position)
{
    auto transform = GetGlobalTransform(body);
    return (transform.invert() * Transform(position)).p();
}

Vec3 PosBodyToWorld(MobilizedBody body, Vec3 position)
{
    auto transform = GetGlobalTransform(body);
    return (transform * Transform(position)).p();
}

Rotation FromDirectionVector(Vec3 direction, CoordinateAxis axis = ZAxis)
{
    return Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);
}

Rotation FromDirectionVector(Vec3 direction_a, Vec3 direction_b)
{
    return Rotation().setRotationFromTwoAxes(UnitVec3(direction_a), ZAxis, UnitVec3(direction_b), ZAxis);
}

Transform TransformWorldToBody(MobilizedBody body, Vec3 position, Vec3 direction, CoordinateAxis axis = ZAxis)
{
    auto transform = GetGlobalTransform(body);
    position = (transform.invert() * Transform(position)).p();
    // rotate into the desired direction
    auto rotation = Rotation().setRotationFromOneAxis(UnitVec3(direction), axis);
    // position = transform.R().invert() * rotation * position;
    return Transform(rotation, position);
}

// Transform TransformWorldToBody(MobilizedBody body, Vec3 position)
// {
//     auto transform = GetGlobalTransform(body);
//     Vec3 position = (transform.invert() * Transform(position)).p();
//     return Transform(rotation, position);
// }

Constraint CreateLink(MobilizedBody &chassis_body, Vec3 chassis_link_pos, MobilizedBody &upright_body, Vec3 upright_link_pos, int variant = 1)
{
    Constraint cons;

    auto dist = (chassis_link_pos - upright_link_pos).norm();

    switch (variant)
    {
    case 0:
        cons = Constraint::SphereOnSphereContact(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), dist / 2.0, upright_body, PosWorldToBody(upright_body, upright_link_pos), dist / 2.0, false);
        break;
    case 1:
        cons = Constraint::Rod(chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), upright_body, PosWorldToBody(upright_body, upright_link_pos), dist);
        break;
    }

    return cons;
}

MobilizedBody CreateSteeringRack(JSON hardpoints, MobilizedBody &chassis_body)
{
    /*
        The steering system consists of:
        - steering rack (slider): lateral w.r.t. chassis
        - pinion (1d velocity constraint): translation to rotational w.r.t. rack
        - steering shaft (revolute): fixed to pinion and rotates w.r.t. chassis
        - intermediate shaft (universal): with respect to steering shaft
        - steering column (universal):
    */

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
    auto steering_rack = MobilizedBody::Slider(chassis_body, Transform(FromDirectionVector(steering_rack_dir, XAxis), PosWorldToBody(chassis_body, steering_rack_pos)), steeringRackInfo, Transform(Vec3(0)));

    // Get steering shaft and column coordinates
    auto pinion_center_at_rack = GetVec3(hardpoints["pinion_center_at_rack"], scale);
    auto intermediate_shaft_forward = GetVec3(hardpoints["intermediate_shaft_forward"], scale); // don't care for now...
    auto intermediate_shaft_rear = GetVec3(hardpoints["intermediate_shaft_rear"], scale);
    auto steeringwheel_center = GetVec3(hardpoints["steeringwheel_center"], scale);

    auto seering_column_dir = steeringwheel_center - intermediate_shaft_rear;

    auto intermediate_shaft_dir = intermediate_shaft_rear - pinion_center_at_rack;
    auto steering_shaft_len = intermediate_shaft_dir.norm();
    auto intermediate_shaft_center = (intermediate_shaft_rear + pinion_center_at_rack) / 2.0;

    // Calculate virtual contact point to achieve desired steering ration
    auto steering_rack_ratio = 0.0024; // [rev/mm]
    auto effective_pinion_radius = (1.0 / steering_rack_ratio) * 1.0 / (2.0 * Pi) / 1000.0;
    auto pinion_virtual_contact_pos = pinion_center_at_rack + effective_pinion_radius * UnitVec3(steering_rack_pos - pinion_center_at_rack);

    // Steering shaft properties
    Body::Rigid pinionInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
    pinionInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, steering_shaft_len / 2.0));

    // Rack pinion constraint, ideally we use a mobilizer here, but the mobilizer we need is not available OOTB
    // ISSUE #753 https://github.com/simbody/simbody/issues/753

    // Attach a revolute mobilizer to the chassis, this will assure a rotational degree of freedom of the ARB w.r.t. the chassis
    auto steering_shaft = MobilizedBody::Revolute(chassis_body, Transform(FromDirectionVector(intermediate_shaft_dir, ZAxis), PosWorldToBody(chassis_body, intermediate_shaft_center)), pinionInfo, Transform());

    // std::cout << TransformWorldToBody(chassis_body, intermediate_shaft_center, intermediate_shaft_dir, ZAxis) << std::endl;
    // std::cout << "ALT TRANS:" << std::endl;
    // std::cout << Transform(FromDirectionVector(intermediate_shaft_dir, ZAxis), PosWorldToBody(chassis_body, intermediate_shaft_center)) << std::endl;

    auto steering_shaft2 = MobilizedBody::Revolute(steering_shaft, TransformWorldToBody(steering_shaft, intermediate_shaft_rear, intermediate_shaft_dir, ZAxis), pinionInfo, TransformWorldToBody(steering_shaft, intermediate_shaft_rear, -seering_column_dir, ZAxis));

    // Add a no slip 1d constraints to constrain the rotation, we need to use the virtual contact position here to get the correct steering ratio
    // Constraint::NoSlip1D(steering_rack, PosWorldToBody(steering_rack, pinion_virtual_contact_pos), UnitVec3(steering_rack_dir), steering_rack, steering_shaft);
    Constraint::NoSlip1D(chassis_body, PosWorldToBody(chassis_body, pinion_virtual_contact_pos), UnitVec3(steering_rack_dir), steering_rack, steering_shaft);

    // This calculation needs to be improved:
    // - We should first find the perpendicular distance between the rack axis and the steering rod axis

    // Steering column body (can incoorporate the steering wheel here if desired)
    // Need to decide whether we really want this or whether a physical interface is already available (i.e. driving simulator)
    // ...
    // Body::Rigid steeringColumInfo(MassProperties(1.0, Vec3(0), UnitInertia(0.01)));
    // steeringColumInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0), XAxis), Vec3(0.0)), DecorativeCylinder(0.01, steering_shaft_len / 2.0));

    // MobilizedBody::Universal(steering_shaft, Transform(PosWorldToBody(steering_shaft, intermediate_shaft_rear)), steeringColumInfo, Transform(Vec3(steering_shaft_len, 0.0, 0.0)));
    // MobilizedBody::Universal(steering_shaft, Transform(PosWorldToBody(steering_shaft, intermediate_shaft_rear)), steeringColumInfo, Transform(PosWorldToBody(steering_shaft, intermediate_shaft_rear)));

    // Debugging steering ratio calculations
    /*
    auto geometric_pinion_radius = (pinion_center_at_rack - steering_rack_pos).norm();
    std::cout << geometric_pinion_radius * 1000.0 << std::endl;
    std::cout << effective_pinion_radius * 1000.0 << std::endl;
    std::cout << (pinion_center_at_rack - pinion_virtual_contact_pos).norm() * 1000.0 << std::endl;
    */

    return steering_rack;
}

void CreateAntiRollbar(JSON hardpoints, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &rocker_body_left, MobilizedBody &rocker_body_right)
{
    // Define arb stiffness
    auto torsional_stiffness = 1.00 * 1000.0 / 180.0 * Pi; // base unit N*mm/deg

    // Describe mass and visualization properties for a generic body.
    Body::Rigid arbInfo(MassProperties(0.1, Vec3(0), UnitInertia(0.1)));

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    auto chassis_frame = chassis_body.getDefaultInboardFrame();

    Vec3 chassis_pos = chassis_frame.p();

    auto arb_bend = GetVec3(hardpoints["arb_bend"], scale);
    auto arb_middle = Vec3(arb_bend[0], 0.0, arb_bend[2]);

    auto arb_droplink = GetVec3(hardpoints["droplink_to_arb"], scale);
    auto droplink_rocker = GetVec3(hardpoints["arblink_to_bellcrank"], scale);
    auto rocker_position = GetVec3(hardpoints["bellcrank_pivot"], scale);

    auto arb_rot_dir = arb_bend - arb_middle;

    auto droplink_dist = (droplink_rocker - arb_droplink).norm();
    arbInfo.addDecoration(Transform(FromDirectionVector(Vec3(0.0, 1.0, 0.0), XAxis)), DecorativeCylinder(0.01, droplink_dist / 2.0));

    // Attach a revolute mobilizer to the chassis, this will assure a rotational degree of freedom of the ARB w.r.t. the chassis
    auto arb_body_left = MobilizedBody::Revolute(chassis_body, Transform(FromDirectionVector(arb_rot_dir, ZAxis), PosWorldToBody(chassis_body, arb_middle)), arbInfo, Transform());

    // Attach the right hand side body
    auto arb_body_right = MobilizedBody::Revolute(chassis_body, Transform(FromDirectionVector(arb_rot_dir.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)), ZAxis), PosWorldToBody(chassis_body, arb_middle)), arbInfo, Transform());

    // Attach the droplink from the rocker body onto to the arb body
    CreateLink(arb_body_left, arb_droplink, rocker_body_left, droplink_rocker);
    CreateLink(arb_body_left, arb_droplink.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)), rocker_body_left, droplink_rocker.elementwiseMultiply(Vec3(1.0, -1.0, 1.0)));

    // TODO: Add stiffness between the bodies
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

    Force::TwoPointLinearSpring(forces, chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), stiffness, x0);
    Force::TwoPointLinearDamper(forces, chassis_body, PosWorldToBody(chassis_body, chassis_link_pos), rocker_body, PosWorldToBody(rocker_body, rocker_link_pos), damping);
}

void CreateSpringDampers(JSON hardpoints, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &upright_body, bool left = true)
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

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    if (!left)
        scale = FlipAxis(scale, 1);

    Vec3 wheel_center = GetVec3(hardpoints["wheel_center"], scale);

    upright.upright = MobilizedBody::Free(chassis_body, Transform(PosWorldToBody(chassis_body, wheel_center)), uprightInfo, Transform(Vec3(0)));

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

MobilizedBody CreateRocker(MultibodySystem &system, JSON hardpoints, GeneralForceSubsystem &forces, MobilizedBody &chassis_body, MobilizedBody &upright_body, bool left = true)
{
    Body::Rigid rockerInfo(MassProperties(0.1, Vec3(0), UnitInertia(0.001)));
    // rockerInfo.addDecoration(Transform(), DecorativeSphere(0.025));

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;

    if (!left)
        scale = FlipAxis(scale, 1);

    auto rocker_pos = GetVec3(hardpoints["bellcrank_pivot"], scale);
    auto rocker_dir = GetVec3(hardpoints["bellcrank_pivot_orient"], scale) - rocker_pos;

    auto transform_rocker_body = Transform(RotationFromDirection(rocker_dir), PosWorldToBody(chassis_body, rocker_pos));
    auto rocker = MobilizedBody::Pin(chassis_body, transform_rocker_body, rockerInfo, Transform(Vec3(0)));

    auto rocker_link_pos = GetVec3(hardpoints["prod_to_bellcrank"], scale);
    auto upright_link_pos = GetVec3(hardpoints["prod_outer"], scale);

    auto dist = (rocker_link_pos - upright_link_pos).norm();

    CreateLink(upright_body, upright_link_pos, rocker, rocker_link_pos);

    return rocker;
}

JSON LoadParameters(std::string filename)
{
    std::ifstream f(filename);
    std::stringstream buffer;
    buffer << f.rdbuf();
    f.close();
    return JSON::Load(buffer.str());
}

int main()
{
    // Define the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -ZAxis, 9.81);

    // Describe mass and visualization properties for a generic body.
    Body::Rigid bodyInfo(MassProperties(1.0, Vec3(0), UnitInertia(1)));
    bodyInfo.addDecoration(Transform(), DecorativeSphere(0.1));

    // Load parameters
    auto data = LoadParameters("../parameters/kinematics.json");

    Vehicle vehicle;

    // Create chassis body
    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;
    auto chassis_pos = (GetVec3(data["front"]["wheel_center"], scale) + GetVec3(data["rear"]["wheel_center"], scale)) / 2.0;
    chassis_pos = Vec3(chassis_pos[0], 0.0, 0.35);

    vehicle.chassis.body = MobilizedBody::Weld(matter.Ground(), Transform(chassis_pos), bodyInfo, Transform(Vec3(0)));

    for (int i = 0; i < 2; i++)
    {
        bool left = i == 0 || i == 2;
        JSON hardpoints = i < 2 ? data["front"] : data["rear"];

        if (i == 0)
        {
            // Generate steering rack
            vehicle.steering_rack = CreateSteeringRack(hardpoints, vehicle.chassis.body);
        }

        vehicle.upright[i] = CreateUpright(hardpoints, vehicle.chassis.body, left);
        if (i < 2)
            vehicle.geometry[i] = CreateMultiLink(hardpoints, vehicle.chassis.body, vehicle.steering_rack, vehicle.upright[i].upright, left);
        else
            vehicle.geometry[i] = CreateMultiLink(hardpoints, vehicle.chassis.body, vehicle.chassis.body, vehicle.upright[i].upright, left);

        vehicle.rocker[i] = CreateRocker(system, hardpoints, forces, vehicle.chassis.body, vehicle.upright[i].upright, left);
        CreateSpringDampersInboard(hardpoints, forces, vehicle.chassis.body, vehicle.rocker[i], left);

        if (!left)
        {
            // Attach ARB assemblies
            CreateAntiRollbar(hardpoints, forces, vehicle.chassis.body, vehicle.rocker[i - 1], vehicle.rocker[i - 0]);
        }
    }

    bool skip_viz = false;
    bool test_model = true;

    auto t_end = 20.0;

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
    RungeKutta3Integrator integ(system);

    integ.setFixedStepSize(1.0e-3);
    TimeStepper ts(system, integ);
    ts.initialize(state);

    if (test_model)
        t_end = 0.001;

    auto start = high_resolution_clock::now();
    ts.stepTo(t_end);
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    std::cout << "Time taken by function: "
              << duration.count() / 1000.0 << " [ms] " << t_end / (duration.count() / 1.0e6) << " [RTF] " << std::endl;
}