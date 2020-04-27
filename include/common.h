/*
 * Copyright 2020 WJZ_Planner, Jiazheng Wu
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WJZ_PLANNER_COMMON_H
#define WJZ_PLANNER_COMMON_H

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <cassert>
#include <random>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "ros/console.h"
#include "special_define.h"
#include "nav_msgs/OccupancyGrid.h"

namespace wjz_planner {

    class Pose_6d{
    public:
        Pose_6d();
        Pose_6d(const Pose_6d &obj);

        R8 x;   //m
        R8 y;   //m
        R8 yaw; //rad
        R8 v;   //m/s
        R8 w;   //rad/s
        R8 a;   //m/s2
    };
}

#endif //WJZ_PLANNER_COMMON_H
