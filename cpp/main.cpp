#include "Simbody.h"

#include "chassis.hpp"

#include "utilities/json.hpp"

#include <chrono>
#include <string>

using namespace json;

using namespace SimTK;

using namespace std::chrono;

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