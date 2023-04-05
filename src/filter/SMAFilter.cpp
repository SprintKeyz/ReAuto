#include "reauto/filter/SMAFilter.hpp"

namespace reauto {
namespace filter {
SMAFilter::SMAFilter(int size) {
    m_sampleSize = size;
}

double SMAFilter::calculate(double value) {
    m_sum += value;

    if (m_values.size() >= m_sampleSize) {
        m_sum -= m_values.front();
        m_values.pop_back();
    }

    m_values.push_back(value);
    m_mean = m_sum / m_values.size();
    return m_mean;
}
}
}