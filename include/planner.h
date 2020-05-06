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

#ifndef WJZ_PLANNER_PLANNER_H
#define WJZ_PLANNER_PLANNER_H

#include "common.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#define Dijkstra 1
#define A_Star 2
#define RPM 3
#define RRT 4
#define RRT_Star 5
#define Sweep 6
#define Dwa 7
#define State_Lattice 8
#define Reeds_Shepp 9
#define Map_Refer 10

#define manhattan 1
#define Euclidean 2

namespace wjz_planner {

    class graph_v{
    public:
        graph_v();

        std::vector<U4> corr_index;//f
        std::vector<R8> corr_dis;
    };

    class Plan_Param{
    public:
        Plan_Param();

        //1.for all algorithm
        X1 plan_mode;
        X1 obstacle_prob;  //0-100, bigger than this number is obstacle
        U4 reduce_scale;   //Reduce resolution scale 1:none
        B1 use_local_plan; //whether use local plan

        //2.for Dijkstra_2D ,RPM
        //none

        //3.for A_Star, RPM
        R8 i_w;            //indicator function weight
        X1 i_mode;         //indicator function mode:   1:manhattan  2:Euclidean

        //4.kdtree config for RPM,RRT,RRT_Star
        B1 KNsearch;       //nearest k search or radiusSearch
        U4 K;              //K in nearest k search
        R8 radius;         //radius in radius search, (m)

        //5.for RPM
        X1 s_mode;         //search mode:  1:Dijkstra 2:A_Star
        B1 obstacle_avoid; //if true for nearest 9 point no block
        U4 seed_num;       //random point num
        U4 k_choice;       //search nearest k point to construct edge

        //6.for RRT
        R8 stop_dis;      //stop distance to final
        U4 max_try_times; //max RRT times;
        R8 max_distance;  //leaf max extend distance

        //7.for RRT_Star
        R8 set_dis;       //x_rand search dis;

        //8.for Sweep Robot
        X1 block_Search;  //if block,use what algorithm to find the nearest point

        //9.for Map Refer Simultion
        R8 sim_v;
        R8 sim_dt;
        R8 sim_wb;        //simulation wheel base

        //10.for DWA in diff model
        R8 w_scale;       //w scale
        R8 w_interval;    //w interval
        R8 v_scale;       //v scale
        R8 v_interval;    //v interval
        R8 dt;            //measure time
        R8 min_block_dis; //min block distance,lower than this ,return false
        U4 init_index;    //search init index
        U4 end_index;     //end init index
        R8 ap,bp,cp;      //loss function param
        R8 out_interval;  //output time interval

        //11.for state lattice in auto car
        R8 path_v;
        R8 path_dt;
        R8 l1_scale;
        R8 l1_scalep;
        R8 l1_intv;
        R8 s1_scale;
        R8 s1_scalep;
        R8 s1_intv;
        R8 s1d_scale;
        R8 s1d_scalep;
        R8 s1d_intv;
        R8 t1_scale;
        R8 t1_scalep;
        R8 t1_intv;
        R8 sample_intv;
        R8 af,bf,cf;
        R8 out_t_intv;

        //12.for Reeds shepp in auto car
        Pose_6d stop_pose;//stop pose
        R8 max_search;    //max search index
        R8 wheel_base;    //wheel base /m
        R8 max_steer;     //max steer angle /rad
        R8 t_interval;    //t interval for detect
        R8 idling;        //must be negative
        R8 yaw_bias;      //max stop yaw bias
        R8 sx_bias;       //max stop x bias
        R8 sy_bias;       //max stop negative y bias
        R8 res_bias;      //max stop res bias
        R8 out_t;         //output t interval
        B1 pre_intp;      //use pre_interpolation
        U4 intp_scale;    //interpolation scale
    };

    class Planner{
    public:
        Planner();
        ~Planner();

        //0.auxiliary functions and variable
        std::vector<U4> index_list;
        void DFS(U4 start_i,U4 end_i,U4* pre);
        R8 GetDistance(I4 x1,I4 y1,I4 x2,I4 y2,X1 mode, X1 i_w);
        void Reduce_Resolution(nav_msgs::OccupancyGrid &map_before,nav_msgs::OccupancyGrid &map_now,U4 scale);
        std::vector<Pose_6d> Linear_Interpolation(std::vector<Pose_6d> &refer_line,U4 scale);
       
        //for usage git clone spline.h
        std::vector<Pose_6d> Cubic_Spline(std::vector<Pose_6d> &refer_line);

        //1.main fuction
        std::vector<Pose_6d> GetPlan(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                     Pose_6d &final_pose,Plan_Param &param);

        std::vector<Pose_6d> GetPlan(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                     Plan_Param &param,std::vector<Pose_6d> &refer_line);

        //2.global geometrical planning
        std::vector<Pose_6d> Dijkstra_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                         Pose_6d &final_pose,Plan_Param &param);

        std::vector<Pose_6d> A_Star_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                       Pose_6d &final_pose,Plan_Param &param);

        std::vector<Pose_6d> RPM_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                    Pose_6d &final_pose,Plan_Param &param);

        std::vector<Pose_6d> RRT_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                    Pose_6d &final_pose,Plan_Param &param);

        std::vector<Pose_6d> RRT_Star_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                         Pose_6d &final_pose,Plan_Param &param);

        std::vector<Pose_6d> Sweep_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                         Pose_6d &final_pose,Plan_Param &param);

        //just for auto car simulation,you can also read from file
        std::vector<Pose_6d> Map_Refer_Simulation(Pose_6d &init_pose,Plan_Param &param);

        //2.local plan,for performance,you should extract map first
        //only for diff robot,sweep robot
        std::vector<Pose_6d> Dwa_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                    Plan_Param &param,std::vector<Pose_6d> &refer_line);

        //only for auto drive, change lane, in frenet,just use map refer line
        std::vector<Pose_6d> State_Lattice_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                              Plan_Param &param,std::vector<Pose_6d> &refer_line);

        //only for auto drive,mainly for back car final pose is in plan param,the line must be pre-processed
        std::vector<Pose_6d> Reeds_Shepp_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                            Plan_Param &param,std::vector<Pose_6d> &refer_line);

        //3.undone algorithm here,
        //3.1 MPC: model predictive planner,see my controller program in github
        //3.2 potential field: rarely use now, raplaced by probabilistic planner
        //3.3 RRT connect: unnecessary for driverless car,but for some robot arm,it is OK
        //3.4 EM Planner: due to copyright,you can find it yourself,but its performance is worse than MPC planner
        //3.5 hybrid A*: Reeds shepp with A*,use in auto drive,but global path usually use map refer line
        //3.6 dubin: rarely use, as many car can back
        //3.7 RRT+Reeds_shepp/dubin: rarely use now
        //3.8 D*: need strong computing capability

        //4.Mainstream using
        //4.1 Dijkstra/A* + Dwa  sweep robot
        //4.2 map_refer_line + state_lattice/MPC/EM planner/Reeds shepp  auto taxi
        //4.3 for detect whether it can reach,use RRT for sweep robot.
    };

}

#endif //WJZ_PLANNER_PLANNER_H
