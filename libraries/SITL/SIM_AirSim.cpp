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
	Simulator Connector for AirSim
*/

#include "SIM_AirSim.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>
#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

AirSim::AirSim(const char *frame_str) :
	Aircraft(frame_str),
	sock(true)
{
	printf("Starting SITL Airsim\n");
}

/*
	Create & set in/out socket
*/
void AirSim::set_interface_ports(const char* address, const int port_in, const int port_out)
{
	if (!sock.bind("0.0.0.0", port_in)) {
		printf("Unable to bind Airsim sensor_in socket at port %u - Error: %s\n",
				 port_in, strerror(errno));
		return;
	}
	printf("Bind SITL sensor input at %s:%u\n", "127.0.0.1", port_in);
	sock.set_blocking(false);
	sock.reuseaddress();

	airsim_ip = address;
	airsim_control_port = port_out;
	airsim_sensor_port = port_in;

	printf("AirSim control interface set to %s:%u\n", airsim_ip, airsim_control_port);
}

// /* Mark Cline */
// ------struct is for reference only ----
// struct RotorControlMessage {
//     // AirSim packet format for controlling each motor
//     uint16_t pwm[4]; // 4 is number of motors
//     uint16_t speed, direction, turbulence; // wind speed, dir, & turbulence
// }
// ---------------------------------------
/*
	Decode and send servos
*/
int i = 0;
void AirSim::send_servos(const struct sitl_input &input)
{
	servo_packet pkt{0};

    pkt.pwm[0] = input.servos[2];
    pkt.pwm[1] = input.servos[3];
    pkt.pwm[2] = input.servos[1];
    pkt.pwm[3] = input.servos[0];
    pkt.speed = 0;
    pkt.direction = 0;
    pkt.turbulence = 0;

    if (pkt.pwm[0] > 1000) {
        //printf("motor speed: %i\n", pkt.pwm[0]);
        // pkt.pwm[0] = 1500;//input.servos[2];
        // pkt.pwm[1] = 1500;//input.servos[3];
        // pkt.pwm[2] = 1500 + i;//input.servos[1];
        // pkt.pwm[3] = 1500 + i++;
    }

	ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
	if (send_ret != sizeof(pkt)) {
		if (send_ret <= 0) {
			printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
					 airsim_ip, airsim_control_port, strerror(errno), send_ret);
		} else {
			printf("Sent %ld bytes instead of %ld bytes\n", send_ret, sizeof(pkt));
		}
	}
}

/*
	Receive new sensor data from simulator
	This is a blocking function
*/
#pragma pack(push, 1) 
// mark edit
struct SensorMessage {
        // this is the packet sent by the simulator
        // to the APM executable to update the simulator state
        // All values are little-endian
        uint64_t timestamp;
        double latitude, longitude; // degrees
        double altitude;  // MSL
        double heading;   // degrees
        double speedN, speedE, speedD; // m/s
        double xAccel, yAccel, zAccel;       // m/s/s in body frame
        double rollRate, pitchRate, yawRate; // degrees/s/s in earth frame
        double rollDeg, pitchDeg, yawDeg;    // euler angles, degrees
        double airspeed; // m/s
        //double pressue, baro_alt; // barometer sensor data
        uint32_t magic; // 0x4c56414f
};
#pragma pack(pop)

void AirSim::recv_fdm()
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    while (ret <= 0) {
        //printf("No sensor message received - %s\n", strerror(errno));
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    }
    // mark edit
    SensorMessage packet;
    if (ret > 0) {
        memcpy(&packet, &sensor_buffer[sensor_buffer_len], ret);
    }
    //printf("sensor message: %f, %f, %f\n\n", packet.rollRate, packet.pitchRate, packet.yawRate);
    // end edit

    AP_Baro *barometer = AP_Baro::get_singleton();
    barometer->get_altitude();
    accel_body = Vector3f(packet.xAccel,// state.imu.linear_acceleration[0],
                          -packet.yAccel,// state.imu.linear_acceleration[1],
                          packet.zAccel);// state.imu.linear_acceleration[2]);

    gyro = Vector3f(packet.rollRate,// state.imu.angular_velocity[0],
                    packet.pitchRate,// state.imu.angular_velocity[1],
                    -packet.yawRate);// state.imu.angular_velocity[2]);

    velocity_ef = Vector3f(packet.speedN,// state.velocity.world_linear_velocity[0],
                           packet.speedE,// state.velocity.world_linear_velocity[1],
                           packet.speedD);// state.velocity.world_linear_velocity[0]);

    double fractpart, intpart;
    fractpart = modf(packet.latitude, &intpart);
    location.lat = ((int)intpart * 1.0e7) + (int)(fractpart * 1.0e7);// state.gps.lat * 1.0e7;
    fractpart = modf(packet.longitude, &intpart);
    location.lng = ((int)intpart * 1.0e7) + (int)(fractpart * 1.0e7);//state.gps.lon * 1.0e7;
    location.alt = (double)packet.altitude * 100;// state.gps.alt * 100.0f;
    // gcs().send_text(MAV_SEVERITY_INFO, "GPS location = %d, %d, %d", location.lat, location.lng, location.alt);

    dcm.from_euler(packet.rollDeg, packet.pitchDeg, -packet.yawDeg);// state.pose.roll, state.pose.pitch, state.pose.yaw);

    if (last_timestamp) {
        int deltat = packet.timestamp - last_timestamp;// state.timestamp - last_timestamp;
        time_now_us += deltat;

        if (deltat > 0 && deltat < 100000) {
            if (average_frame_time < 1) {
                average_frame_time = deltat;
            }
            average_frame_time = average_frame_time * 0.98 + deltat * 0.02;
        }
    }

    //scanner.points = state.lidar.points;

    // rcin_chan_count = state.rc.rc_channels.length < 8 ? state.rc.rc_channels.length : 8;
    // for (uint8_t i=0; i < rcin_chan_count; i++) {
    //     rcin[i] = state.rc.rc_channels.data[i];
    // }

    last_timestamp = packet.timestamp;//state.timestamp;
}

/*
  update the AirSim simulation by one time step
*/
void AirSim::update(const struct sitl_input &input)
{
	send_servos(input);
    recv_fdm();

    // update magnetic field
    update_mag_field_bf();
}
