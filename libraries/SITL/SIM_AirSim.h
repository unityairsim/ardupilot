/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/* 
	Simulator connector for Airsim: https://github.com/Microsoft/AirSim
*/

#pragma once

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

/* 
	Airsim Simulator
*/

class AirSim : public Aircraft {
public:
	AirSim(const char *frame_str);

	/* update model by one time step */
	void update(const struct sitl_input &input) override;

	/* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new AirSim(frame_str);
    }

    /*  Create and set in/out socket for Airsim simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
	/*
		rotor control packet sent by Ardupilot
	*/
	static const int kArduCopterRotorControlCount = 4;

	struct servo_packet {
		uint16_t pwm[kArduCopterRotorControlCount];
        uint16_t speed, direction, turbulence; // wind speed, dir, & turbulence // mark edit added line
	};

	// default connection_info_.ip_address
	const char *airsim_ip = "127.0.0.1";

	// connection_info_.ip_port
	uint16_t airsim_sensor_port = 9003;

	// connection_info_.sitl_ip_port
	uint16_t airsim_control_port = 9002;

	SocketAPM sock;

    double average_frame_time;
    uint64_t frame_counter;
    uint64_t last_frame_count;
    uint64_t last_timestamp;

	void send_servos(const struct sitl_input &input);
	void recv_fdm();

	// buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[65000];
    uint32_t sensor_buffer_len;
};

}
