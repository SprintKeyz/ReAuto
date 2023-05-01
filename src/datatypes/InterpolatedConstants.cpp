#include "reauto/datatypes/InterpolatedConstants.hpp"

InterpolatedConstants::InterpolatedConstants(std::vector<IPIDConstants> gainTable)
{
    if (gainTable.size() < 2)
    {
        // not enough constants for an interpolated
        // table!

        // we use 0, 0, 0 as a default, so don't print for that case

        if (gainTable[0].kP != 0 || gainTable[0].kI != 0 || gainTable[0].kD != 0) {
            std::cout << "[ReAuto] Not enough points for interpolated constants!" << std::endl;
            std::cout << "[ReAuto] Falling back to default constants..." << std::endl;
        }

        m_isSimpleConstant = true;
        m_constants = { gainTable[0].kP, gainTable[0].kI, gainTable[0].kD };
        return;
    }

    else if (gainTable.size() == 2)
    {
        // wish we had more
        std::cout << "[ReAuto] WARN: Three points are recommended for interpolated constants." << std::endl;
    }

    for (auto constants : gainTable)
    {
        m_pLUT.addPoint(constants.error, constants.kP);
        m_iLUT.addPoint(constants.error, constants.kI);
        m_dLUT.addPoint(constants.error, constants.kD);
    }

    m_pLUT.create();
    m_iLUT.create();
    m_dLUT.create();
}

void InterpolatedConstants::setGainTable(std::vector<IPIDConstants> gainTable)
{
    if (gainTable.size() < 2)
    {
        // not enough constants for an interpolated
        // table!

        std::cout << "[ReAuto] Not enough points for interpolated constants!" << std::endl;
        std::cout << "[ReAuto] Falling back to default constants..." << std::endl;

        m_isSimpleConstant = true;
        m_constants = { gainTable[0].kP, gainTable[0].kI, gainTable[0].kD };
        return;
    }

    else if (gainTable.size() == 2)
    {
        // wish we had more
        std::cout << "[ReAuto] WARN: Three points are recommended for interpolated constants." << std::endl;
    }

    for (auto constants : gainTable)
    {
        m_pLUT.addPoint(constants.error, constants.kP);
        m_iLUT.addPoint(constants.error, constants.kI);
        m_dLUT.addPoint(constants.error, constants.kD);
    }

    m_pLUT.create();
    m_iLUT.create();
    m_dLUT.create();
}

PIDConstants InterpolatedConstants::get(double error)
{
    if (m_isSimpleConstant)
        return m_constants;

    return {
        m_pLUT.get(error),
        m_iLUT.get(error),
        m_dLUT.get(error) };
}