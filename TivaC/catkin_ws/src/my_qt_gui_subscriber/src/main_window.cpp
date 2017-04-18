/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#endif

#include <string>
#include <sstream>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/my_qt_gui_subscriber/main_window.hpp"
#include "../include/my_qt_gui_subscriber/qnode.hpp"

#include "adc_joystick_msg/ADC_Joystick.h"
#include "stepper_msg/Stepper_Control.h"

#define CONTROL_MODE_OFF      0
#define CONTROL_MODE_HOME     1
#define CONTROL_MODE_X_AXIS   2
#define CONTROL_MODE_Y_AXIS   3
#define CONTROL_MODE_X_POSE   4
#define CONTROL_MODE_Y_POSE   5
#define CONTROL_MODE_GOTO     6

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace my_qt_gui_subscriber {

using namespace Qt;


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ros::init(argc,argv,"my_qt_gui_subscriber");
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
    //Button Publish
    //ros::init(argc,argv,"my_qt_gui_publisher");
    //ros::NodeHandle n;
    //QObject::connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(sendMsg()));
    //chatter_publisher = n.advertise<adc_joystick_msg::ADC_Joystick>("adc_joystick", 1000);



}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "my_qt_gui_subscriber");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "my_qt_gui_subscriber");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::on_pushButton_clicked()

{

    ui.stackedWidget->setCurrentIndex(1);



}


void MainWindow::on_pushButton_2_clicked()

{

    ui.stackedWidget->setCurrentIndex(2);

}

void MainWindow::on_pushButton_3_clicked()
{
    stepper_msg::Stepper_Control smsg;
    smsg.control_mode = CONTROL_MODE_HOME;
    ros::NodeHandle n;
    chatter_publisher = n.advertise<stepper_msg::Stepper_Control>("/TUhand/Tendon1Stepper/control", 1000);
    chatter_publisher.publish(smsg);
}


void MainWindow::on_pushButton_4_clicked()

{

    ui.stackedWidget->setCurrentIndex(3);

}


void MainWindow::on_pushButton_5_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);

    ui.label_4->setText("Current Pose: Pose 1");

}


void MainWindow::on_pushButton_6_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);

    ui.label_4->setText("Current Pose: Pose 2");

}


void MainWindow::on_pushButton_7_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);

    ui.label_5->setText("Current Actuator: Tendon Actuator 1");

}


void MainWindow::on_pushButton_8_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);

    ui.label_5->setText("Current Actuator: Tendon Actuator 2");

}


void MainWindow::on_pushButton_9_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);

    ui.label_5->setText("Current Actuator: Wrist Actuator");

}


void MainWindow::on_pushButton_10_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);


}


void MainWindow::on_pushButton_11_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);

}


void MainWindow::on_pushButton_12_clicked()

{

    ui.stackedWidget->setCurrentIndex(0);
}

}  // namespace my_qt_gui_subscriber

