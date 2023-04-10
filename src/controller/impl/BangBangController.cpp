#include "reauto/controller/impl/BangBangController.hpp"

#include <cmath>

namespace reauto {
namespace controller {

BangBangController::BangBangController(double exitError) {
    m_exitError = exitError;
}

void BangBangController::setTarget(double target, bool resetController) {
    m_target = target;
    m_error = target;
}

double BangBangController::calculate(double error) {
    m_error = error;

    if (fabs(m_error) > m_exitError) {
        if (m_error > 0) {
            return 127;
        }
        else {
            return -127;
        }
    }

    else {
        return 0;
    }
}

bool BangBangController::settled() {
    return fabs(m_error) < m_exitError;
}
}
}