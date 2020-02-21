#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "JHPWMPCA9685.h"

#define NUM_OUT 16
#define NEUTRAL 369
#define FULL_BACK 245
#define FULL_FORWARD 492

PCA9685 *driver;
char flag = 0;

double map(double x, double in_min, double in_max, double out_min, double out_max){
    return (x-in_min)*(out_max-out_min) / (in_max-in_min) + out_min;
}

void callback(const geometry_msgs::Twist& msg){
    if(msg.linear.x < 0 && flag == 0){
		flag = 1;
		for(int i = 0; i<2; i++){
			driver->setPWM(i, 0, NEUTRAL);
		}
		//sleep(2);
    }else if(msg.linear.x >= 0){
		flag = 0;
	}
    for(int i = 0; i<2; i++){
    	driver->setPWM(i, 0, map(msg.linear.x, -1.5, 1.5, FULL_BACK, FULL_FORWARD));
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_driver");
    
    driver = new PCA9685();
    int err = driver->openPCA9685();
    if(err < 0){
		ROS_INFO_STREAM("Error: " << driver->error);
    }else{
		ROS_INFO_STREAM("PCA9685 Addr: " << driver->kI2CAddress);
    }
    ROS_INFO("Opening PCA9685");
    driver->setAllPWM(0, 0);
    driver->reset();
    driver->setPWMFrequency(60);

    for(int i = 0; i<NUM_OUT; i++){
	driver->setPWM(i, 0, 368);
    }

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, callback);
    ros::spin();
    
    // Turn them all off when we're done
    for(int i = 0; i<NUM_OUT; i++){
	driver->setPWM(i, 0, 368);
    }

    driver->closePCA9685();

    return 0;
}
