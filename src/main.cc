#include "mainrun.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "WJZ_Planner");
    wjz_planner::MainRun Main_Running;
    Main_Running.Run();
    ros::spin();
    return 0;
}

