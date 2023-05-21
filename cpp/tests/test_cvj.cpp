#include "Simbody.h"

#include <chrono>
#include <string>

using namespace SimTK;

using namespace std::chrono;

class PositionReporter : public PeriodicEventReporter
{
public:
    PositionReporter(const MultibodySystem &system, const MobilizedBody &wheel, const MobilizedBody &driveshaft, const MobilizedBody &body, Real interval) : PeriodicEventReporter(interval), system(system), wheel(wheel), body(body), driveshaft(driveshaft)
    {
    }

    // Show x-y position of the pendulum weight as a function of time.
    void handleEvent(const State &state) const override
    {
        system.realize(state, Stage::Position);
        Vec3 pos = body.getBodyOriginLocation(state);
        Vec3 vel = body.getBodyOriginVelocity(state);

        auto omg = wheel.getMobilizerVelocity(state).get(0);
        auto omg2 = driveshaft.getMobilizerVelocity(state).get(0);

        std::cout << state.getTime() << "\t" << pos << "\t" << vel << "\t" << omg << "\t" << omg2 << std::endl;
    }

private:
    const MultibodySystem &system;
    const MobilizedBody &body;
    const MobilizedBody &wheel;
    const MobilizedBody &driveshaft;
};

int main()
{
    bool test_model = false;
    bool skip_viz = false;

    bool expand = false;

    // Define the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -ZAxis, 9.81);

    auto ground = matter.Ground();

    // data logger
    // system.addEventReporter(new PositionReporter(system, vehicle.getAxle(1).getWheel(1), vehicle.getAxle(1).getDriveShaft(1), vehicle.getChassis(), 0.1));

    auto driveshaft_mass = MassProperties(0.1, Vec3(0), Inertia(Vec3(0.001, 0.001, 0.001)));
    Body::Rigid driveShaftBodyInfo(driveshaft_mass);

    auto base_joint = MobilizedBody::Pin(ground, driveShaftBodyInfo);
    auto input_shaft = MobilizedBody::Weld(base_joint, Transform(Vec3(0.0, 0.0, -1.0)), driveShaftBodyInfo, Transform());

    auto rotate_x_90 = Rotation().setRotationFromAngleAboutX(Pi / 2);
    auto rotate_y_90 = Rotation().setRotationFromAngleAboutY(Pi / 2);
    auto rotate_y_180 = Rotation().setRotationFromAngleAboutY(Pi);

    auto rotate_joint_x = Rotation().setRotationFromAngleAboutX(45.0 / 180.0 * Pi);

    auto r_offset = Vec3(0.0, 0.0, 0.0);
    if (expand)
        r_offset = Vec3(0.0, 0.0, -1.0);

    auto rev_x_1 = MobilizedBody::Pin(input_shaft, Transform(rotate_x_90, r_offset), driveShaftBodyInfo, Transform());
    auto rev_y_2 = MobilizedBody::Pin(rev_x_1, Transform(rotate_y_90, r_offset), driveShaftBodyInfo, Transform()); // rotate_joint_z
    auto rev_x_3 = MobilizedBody::Pin(rev_y_2, Transform(rotate_y_90, r_offset), driveShaftBodyInfo, Transform());
    auto drive_shaft = MobilizedBody::Weld(rev_x_3, Transform(rotate_x_90 * rotate_y_180, Vec3(0.0, -1.0, 0.0)), driveShaftBodyInfo, Transform());

    auto final_bearing = MobilizedBody::Pin(drive_shaft, driveShaftBodyInfo);

    // calculate equiv. transform
    auto final_trans = Transform(rotate_joint_x, Vec3(0.0, 0.0, -1.0)) * Transform(Vec3(0.0, 0.0, -1.0));

    std::cout << rotate_joint_x << final_trans.p() << std::endl;

    // test body
    // MobilizedBody::Weld(ground, final_trans, driveShaftBodyInfo, Transform());

    Constraint::Weld(ground, final_trans, final_bearing, Vec3(0));

    // force steady rotation
    Motion::Steady(base_joint, 10.0);

    // to check
    // rev_y_2.setDefaultAngle(Pi / 4);

    auto step_size = 1.0e-3;
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

    // Simulate for 20 seconds.
    // RungeKuttaMersonIntegrator integ(system);
    // RungeKutta3Integrator integ(system);
    // integ.setFixedStepSize(step_size);

    SemiExplicitEulerIntegrator integ(system, step_size);

    TimeStepper ts(system, integ);
    ts.initialize(state);

    if (test_model)
        t_end = 0.001;

    ts.stepTo(t_end);
}