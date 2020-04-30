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

    R8 yaw_sub(R8 yaw1,R8 yaw2){
        R8 dyaw;
        while(yaw1 >= 2*TT){
            yaw1 -= 2*TT;
        }
        while(yaw1 < 0){
            yaw1 += 2*TT;
        }
        while(yaw2 >= 2*TT){
            yaw2 -= 2*TT;
        }
        while(yaw2 < 0){
            yaw2 += 2*TT;
        }

        dyaw = yaw1 - yaw2;
        while(dyaw < -TT){
            dyaw += 2*TT;
        }
        while(dyaw > TT){
            dyaw -= 2*TT;
        }
        return dyaw;
    }

    R8 yaw_add(R8 yaw1,R8 yaw2){
        R8 ayaw;
        while(yaw1 >= 2*TT){
            yaw1 -= 2*TT;
        }
        while(yaw1 < 0){
            yaw1 += 2*TT;
        }
        while(yaw2 >= 2*TT){
            yaw2 -= 2*TT;
        }
        while(yaw2 < 0){
            yaw2 += 2*TT;
        }

        ayaw = yaw1 + yaw2;
        ayaw = ayaw>=2*TT?(ayaw-2*TT):ayaw;
        return ayaw;
    }

    R8 get_angle(R8 y1,R8 x1,R8 y0,R8 x0){
        R8 dy = y1-y0;
        R8 dx = x1-x0;
        R8 th = atan(dy/dx);
        R8 angle = 0;
        if(dy >= 0 && dx >= 0){
            angle = th;
        }
        if(dy >= 0 && dx < 0){
            angle = TT + th;
        }
        if(dy < 0 && dx < 0){
            angle = TT + th;
        }
        if(dy < 0 && dx >= 0){
            angle = 2*TT + th;
        }
        return angle;
    }

    R8 ENU2RC(R8 yaw){
        R8 cal_yaw=0;
        yaw = DEG2RAD(yaw);
        if(yaw >= DEG2RAD(90)){
            cal_yaw = DEG2RAD(360) - yaw + DEG2RAD(90);
        }
        else if(yaw < DEG2RAD(90)){
            cal_yaw = DEG2RAD(90) - yaw;
        }
        return cal_yaw;
    }

}