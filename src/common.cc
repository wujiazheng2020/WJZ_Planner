#include "common.h"

namespace wjz_planner {

    Pose_6d::Pose_6d(){
        x = 0.0;
        y = 0.0;
        yaw = 0.0;
        v = 0.0;
        w = 0.0;
        a = 0.0;
    }

    Pose_6d::Pose_6d(const Pose_6d &obj){
        x = obj.x;
        y = obj.y;
        yaw = obj.yaw;
        v = obj.v;
        w = obj.w;
        a = obj.a;
    }

}