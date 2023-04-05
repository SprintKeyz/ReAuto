#pragma once

#include <cmath>
#include <vector>

namespace reauto {
namespace filter {
class SMAFilter {
public:
    SMAFilter(int size);
    double calculate(double value);

private:
    int m_sampleSize = 0;
    double m_sum = 0;
    double m_mean = 0;
    std::vector<double> m_values;
};
}
}
