/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <iostream>
#include <std_msgs/UInt16.h>
#include "../include/my_qt_gui_subscriber/qnode.hpp"
#include "adc_joystick_msg/ADC_Joystick.h"
//#include "stepper_msg/Stepper_Status.h"

using namespace std;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace my_qt_gui_subscriber {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"my_qt_gui_subscriber");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
	// Add your ros communications here.
    chatter_subscriber = n.subscribe("adc_joystick", 1000, &QNode::myCallback, this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"my_qt_gui_subscriber");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
	// Add your ros communications here.
    chatter_subscriber = n.subscribe("adc_joystick", 1000, &QNode::myCallback, this);
    //chatter_publisher = n.advertise<adc_joystick_msg::ADC_Joystick>("adc_joystick", 1000);
	start();
	return true;
}

void QNode::run() {
	ros::NodeHandle n;
    chatter_subscriber = n.subscribe("adc_joystick", 1000, &QNode::myCallback, this);
    //chatter_publisher = n.advertise<adc_joystick_msg::ADC_Joystick>("adc_joystick", 1000);
	ros::spin();

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::myCallback(const adc_joystick_msg::ADC_Joystick& message_holder)
{
	//std::stringstream ss;
	//ss << message_holder.data;
    //log(Info, (int)message_holder.x_axis_raw, (int)message_holder.y_axis_raw);
    //ROS_INFO("=============received value is: %f===========",message_holder.data);
  //really could do something interesting here with the received data...but all we do is print it 
} 


void QNode::log( const LogLevel &level, int x, int y) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
                //ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << "X_Axis received value is: " << x << "  Y_Axis received value is: " << y;
				break;
		}
		case(Info) : {
                //ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << "X_Axis received value is: " << x << "  Y_Axis received value is: " << y;
				break;
		}
		case(Warn) : {
                //ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << "X_Axis received value is: " << x << "  Y_Axis received value is: " << y;
				break;
		}
		case(Error) : {
                //ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << "X_Axis received value is: " << x << "  Y_Axis received value is: " << y;
				break;
		}
		case(Fatal) : {
                //ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << "X_Axis received value is: " << x << "  Y_Axis received value is: " << y;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

/**void QNode::sendMsg()
{
    if(n.connected()) {
        adc_joystick_msg::ADC_Joystick msg;
        msg.x_axis_raw = 100;
        msg.y_axis_raw = 100;
        chatter_publisher.publish(msg);
    }
}*/


}  // namespace my_qt_gui_subscriber#include <QtGui>
