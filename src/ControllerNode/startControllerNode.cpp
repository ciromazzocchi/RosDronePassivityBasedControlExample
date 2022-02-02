#include "ros/ros.h"
#include "./Controller.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "passivityController");
    Controller ctrl;
    ctrl.run();
    return 0;
}