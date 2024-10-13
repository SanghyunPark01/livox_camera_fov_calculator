#include "livox_camera_fov_calculator/calc.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_camera_fov");
    ros::NodeHandle nh_("~");

    Calculator calculator(nh_);

    std::thread thSyncData(&Calculator::syncData, &calculator);
    std::thread thCalc(&Calculator::calculateFOV, &calculator);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    signal(SIGINT, mySigintHandler);
    ros::waitForShutdown();

    if(!ros::ok())
    {
        calculator.saveResult();
    }

    return 0;
}