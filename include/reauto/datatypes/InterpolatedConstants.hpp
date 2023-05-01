#pragma once

#include "reauto/datatypes/IPIDConstants.h"
#include "reauto/datatypes/PIDConstants.h"
#include "reauto/math/InterpolatedLUT.hpp"
#include "reauto/datatypes/Point.h"

#include <cmath>

class InterpolatedConstants
{
public:
    InterpolatedConstants(std::vector<IPIDConstants> gainTable);
    void setGainTable(std::vector<IPIDConstants> gainTable);
    PIDConstants get(double error);

private:
    reauto::InterpolatedLUT m_pLUT;
    reauto::InterpolatedLUT m_iLUT;
    reauto::InterpolatedLUT m_dLUT;

    // true if we are simple constants
    bool m_isSimpleConstant = false;
    PIDConstants m_constants;
};