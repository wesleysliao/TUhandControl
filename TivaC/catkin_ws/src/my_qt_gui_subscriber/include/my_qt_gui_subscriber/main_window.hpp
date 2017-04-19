/**
 * @file /include/my_qt_gui_subscriber/main_window.hpp
 *
 * @brief Qt based gui for my_qt_gui_subscriber.
 *
 * @date November 2010
 **/
#ifndef my_qt_gui_subscriber_MAIN_WINDOW_H
#define my_qt_gui_subscriber_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace my_qt_gui_subscriber {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void on_manual_mode_select_clicked();
    void on_manual_return_clicked();
    void on_tendon1_select_clicked();
    void on_tendon2_select_clicked();
    void on_wrist_select_clicked();
    void on_home_all_clicked();
    void on_disable_all_clicked();
    void on_tendon1_return_clicked();
    void on_disable_tendon1_clicked();
    void on_home_tendon1_clicked();
    void on_select_tendon1_clicked();
    void on_tendon2_return_clicked();
    void on_disable_tendon2_clicked();
    void on_home_tendon2_clicked();
    void on_select_tendon2_clicked();
    void on_wrist_return_clicked();
    void on_disable_wrist_clicked();
    void on_home_wrist_clicked();
    void on_select_wrist_clicked();
    /**
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();


    void on_pushButton_4_clicked();


    void on_pushButton_5_clicked();



    void on_pushButton_6_clicked();



    void on_pushButton_7_clicked();



    void on_pushButton_8_clicked();



    void on_pushButton_9_clicked();



    void on_pushButton_10_clicked();



    void on_pushButton_11_clicked();


    void on_pushButton_12_clicked();*/


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        ros::Publisher stepper_publisher;

};

}  // namespace my_qt_gui_subscriber

#endif // my_qt_gui_subscriber_MAIN_WINDOW_H
