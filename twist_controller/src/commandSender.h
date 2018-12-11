
//from 	#include "commonDefines.h"
// CAN bus
const unsigned int SW_CAN_NODE_ID = 30;
const unsigned int SW_CAN_COMMAND_ID = 384;
const unsigned char SW_CAN_FLAG_OVERRIDE = 0x01;
const unsigned char SW_CAN_FLAG_DRIVE_ON = 0x02;
const unsigned char SW_CAN_FLAG_STEER_ON = 0x04;
const unsigned int SW_CAN_CLEAR_ITER_NUM = 100;
const unsigned int SW_CAN_CLEAR_DELAY = 10000; // microsecond



#include "canlibWrapper.h"

#include <unistd.h> // usleep()
#include <cstdlib> // abs()
#include <stdio.h> //printf
#include <ros/ros.h>


/**
 * @brief Sends commands to a car.
 */
class commandSendero
{
    public:
        commandSendero(double _max_car_velocity_change, double _max_car_steer_angle_change);
        void formWriteMSG(double speed, double angle, const char flags);

    private:
		void formWriteMSG(int, int, const char);
		int convertAngle(const double angle);
		int convertVelocity(const double velocity);
        Canlib sw_can;
        unsigned char pdoCounter;
        /// Message flags
        char message_flags;
};
