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

#ifndef WJZ_PLANNER_MAINRUN_H
#define WJZ_PLANNER_MAINRUN_H

#include <thread>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "planner.h"

namespace wjz_planner {

    class MainRun{
    public:
        MainRun();
        ~MainRun();

        void Run();
        void Main_Function();
        void tf_publish_function(R8 time_interval);

    private:
        B1 Main_Running;
        U4 seq;
        U4 tf_seq;
        ros::Time now_time;

        Planner planner;

        ros::NodeHandle node_;
        ros::NodeHandle private_nh_;
        ros::Publisher map_pub;
        ros::Publisher path_pub;
        ros::Publisher lpath_pub;

        std::thread *Main_thread;
        std::thread *tf_thread;
    };

}


#endif //WJZ_PLANNER_MAINRUN_H
