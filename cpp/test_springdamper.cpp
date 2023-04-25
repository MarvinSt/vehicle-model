#include "springdamper.hpp"

int main()
{
    auto lookup_table_var = LookupTable1DVar();

    // Populate the lookup table with some data
    auto num_samples = 48;
    auto max_val = 2.0 * M_PI;

    double idx;
    double val;

    for (int i = 0; i < num_samples; i++)
    {
        double val = max_val * double(i) / double(num_samples);
        lookup_table_var.push_back(val, std::sin(val));
    }

    idx = std::fmod(1.50, max_val);

    val = lookup_table_var.eval(idx);

    std::cout << "u: " << idx << " - " << val << " | " << std::sin(idx) << std::endl;

    auto lookup_table = LookupTable1D();

    // Populate the lookup table with some data
    for (int i = 0; i < num_samples; i++)
    {
        double val = max_val * double(i) / double(num_samples);
        lookup_table.push_back(val, std::sin(val));
    }

    idx = std::fmod(1.50, max_val);

    val = lookup_table.eval(idx);

    std::cout << "u: " << idx << " - " << val << " | " << std::sin(idx) << std::endl;

    return 0;
}