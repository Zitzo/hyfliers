//
//
//
//
//
//

#include <QApplication>
#include "PidTuneGui.h"
#include <ros/ros.h>
#include <thread>

int main(int _argc, char **_argv){
    ros::init(_argc, _argv, "PidTuneGui");
    std::thread spinThread([&](){
        ros::spin();
    });

    QApplication a(_argc, _argv);
    PidTuneGui gui( "/mav_controller/reference", 
                    "/uav_1/mavros/local_position/pose",
                    {   std::pair<std::string, std::string>("X", "/mav_controller/pid_x"),
                        std::pair<std::string, std::string>("Y", "/mav_controller/pid_y"),
                        std::pair<std::string, std::string>("Z", "/mav_controller/pid_z")});
    gui.show();
    
    return a.exec();
}
