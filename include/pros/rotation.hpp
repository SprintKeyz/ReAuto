/**
 * \file pros/rotation.hpp
 * \ingroup cpp-rotation
 *
 * Contains prototypes for functions related to the VEX Rotation Sensor.
 *
 * Visit https://pros.cs.purdue.edu/v5/tutorials/topical/rotation.html to learn
 * more.
 *
 * This file should not be modified by users, since it gets replaced whenever
 * a kernel upgrade occurs.
 *
 * \copyright (c) 2017-2023, Purdue University ACM SIGBots.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * 
 * \defgroup cpp-rotation VEX Rotation Sensor C++ API
 */
#ifndef _PROS_ROTATION_HPP_
#define _PROS_ROTATION_HPP_

#include <cstdint>
#include <iostream>

#include "pros/rotation.h"
#include "pros/device.hpp"

namespace pros {
inline namespace v5 {
/**
 * \addtogroup cpp-rotation
 */
class Rotation : public Device {
	/**
	 * \ingroup cpp-rotation
	 *  @{
	 */

	public:
	explicit Rotation(const std::uint8_t port) : Device(port, DeviceType::rotation) {};

	explicit Rotation(const std::uint8_t port, const bool reverse_flag);

	/**
	 * Reset the Rotation Sensor
	 *
	 * Reset the current absolute position to be the same as the
	 * Rotation Sensor angle.
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \return 1 if the operation was successful or PROS_ERR if the operation
	 * failed, setting errno.
	 */
	virtual std::int32_t reset();

	/**
	 * Set the Rotation Sensor's refresh interval in milliseconds.
	 *
	 * The rate may be specified in increments of 5ms, and will be rounded down to
	 * the nearest increment. The minimum allowable refresh rate is 5ms. The default
	 * rate is 10ms.
	 *
	 * As values are copied into the shared memory buffer only at 10ms intervals,
	 * setting this value to less than 10ms does not mean that you can poll the
	 * sensor's values any faster. However, it will guarantee that the data is as
	 * recent as possible.
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \param rate The data refresh interval in milliseconds
	 * \return 1 if the operation was successful or PROS_ERR if the operation
	 * failed, setting errno.
	 */
	virtual std::int32_t set_data_rate(std::uint32_t rate) const;

	/**
	 * Set the Rotation Sensor position reading to a desired rotation value
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \param position
	 * 		  The position in terms of ticks
	 * \return 1 if the operation was successful or PROS_ERR if the operation
	 * failed, setting errno.
	 */
	virtual std::int32_t set_position(std::uint32_t position) const;

	/**
	 * Reset the Rotation Sensor to a desired rotation value
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \param position
	 * 		  The position in terms of ticks
	 * \return 1 if the operation was successful or PROS_ERR if the operation
	 * failed, setting errno.
	 */
	virtual std::int32_t reset_position(void) const;

	/**
	 * Get the Rotation Sensor's current position in centidegrees
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \return The position value or PROS_ERR if the operation failed, setting
	 * errno.
	 */
	virtual std::int32_t get_position() const;

	/**
	 * Get the Rotation Sensor's current velocity in centidegrees per second
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \param  port
	 * 				 The V5 Rotation Sensor port number from 1-21
	 * \return The
	 value or PROS_ERR_F if the operation failed, setting
	 * errno.
	 */
	virtual std::int32_t get_velocity() const;

	/**
	 * Get the Rotation Sensor's current position in centidegrees
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \return The angle value or PROS_ERR if the operation failed, setting
	 * errno.
	 */
	virtual std::int32_t get_angle() const;

	/**
	 * Set the Rotation Sensor's direction reversed flag
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \param  value
	 * 				 Determines if the direction of the rotational sensor is
	 * 				 reversed or not.
	 *
	 * \return 1 if the operation was successful or PROS_ERR if the operation
	 * failed, setting errno.
	 */
	virtual std::int32_t set_reversed(bool value) const;

	/**
	 * Reverse the Rotation Sensor's direction.
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \return 1 if the operation was successful or PROS_ERR if the operation
	 * failed, setting errno.
	 */
	virtual std::int32_t reverse() const;

	/**
	 * Get the Rotation Sensor's reversed flag
	 *
	 * This function uses the following values of errno when an error state is
	 * reached:
	 * ENXIO - The given value is not within the range of V5 ports (1-21).
	 * ENODEV - The port cannot be configured as an Rotation Sensor
	 *
	 * \return Reversed value or PROS_ERR if the operation failed, setting
	 * errno.
	 */
	virtual std::int32_t get_reversed() const;
	///@}

	/**
	 * This is the overload for the << operator for printing to streams
	 * 
	 * Prints in format(this below is all in one line with no new line):
	 * Rotation [port: rotation._port, position: (rotation position), velocity: (rotation velocity), 
	 * angle: (rotation angle), reversed: (reversed boolean)]
	 */
	friend std::ostream& operator<<(std::ostream& os, const pros::Rotation& rotation);

};

namespace literals {
const pros::Rotation operator"" _rot(const unsigned long long int r);
}  // namespace literals
}
}  // namespace pros

#endif
