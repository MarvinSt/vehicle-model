#include "Simbody.h"

#include "chassis.hpp"

#include "utilities/json.hpp"

#include <chrono>
#include <string>

using namespace json;

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

        std::cout << state.getTime() << "\t" << pos << "\t" << vel << "\t" << omg << "\t" << std::endl;
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
    bool constrain_chassis = false;

    // load parameters
    // auto data = LoadParameters("../parameters/kinematics.json");
    auto data = LoadParameters("../parameters/chassis.json");

    // Define the system.
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forces(system);
    Force::Gravity gravity(forces, matter, -ZAxis, 9.81);

    auto ground = matter.Ground();

    auto scale = Vec3(1.0, 1.0, 1.0) / 1000.0;
    auto vehicle = Chassis(data, scale, forces, ground);

    // push forwards
    // Force::ConstantForce(forces, vehicle.getChassis(), Vec3(0), -Vec3(1.0, 0.5, 0.0) * 100);
    // Force::ConstantTorque(forces, vehicle.getChassis(), Vec3(1.0, 0.0, 1.0) * 20);
    system.addEventReporter(new PositionReporter(system, vehicle.getAxle(1).getWheel(1), vehicle.getAxle(1).getDriveShaft(1), vehicle.getChassis(), 0.1));

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

    auto start = high_resolution_clock::now();
    ts.stepTo(t_end);
    auto stop = high_resolution_clock::now();

    auto duration = duration_cast<microseconds>(stop - start);

    std::cout << "Time taken by function: "
              << duration.count() / 1000.0 << " [ms] " << t_end / (duration.count() / 1.0e6) << " [RTF] " << std::endl;
}