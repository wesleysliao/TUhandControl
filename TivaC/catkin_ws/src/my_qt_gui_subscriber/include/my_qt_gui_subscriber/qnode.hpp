/**
 * @file /include/my_qt_gui_subscriber/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef my_qt_gui_subscriber_QNODE_HPP_
#define my_qt_gui_subscriber_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <std_msgs/UInt16.h>
#include <QThread>
#include <QStringListModel>
#include "adc_joystick_msg/ADC_Joystick.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace my_qt_gui_subscriber {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
        void myCallback(const adc_joystick_msg::ADC_Joystick& message_holder);

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
        void log( const LogLevel &level, int x, int y);
        void sendMsg();

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
        adc_joystick_msg::ADC_Joystick js_msg;
	int init_argc;
	char** init_argv;
	ros::Subscriber chatter_subscriber;
        ros::Publisher chatter_publisher;
    QStringListModel logging_model;
};

}  // namespace my_qt_gui_subscriber

#endif /* my_qt_gui_subscriber_QNODE_HPP_ */
