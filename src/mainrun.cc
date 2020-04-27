#include "mainrun.h"

namespace wjz_planner {

    MainRun::MainRun(){
        Main_Running = true;
        seq = 0;
        tf_seq = 0;
        now_time = ros::Time::now();
    }

    MainRun::~MainRun(){}

    void MainRun::Main_Function(){
        //1.init map param
        nav_msgs::OccupancyGrid map_now;
        map_now.info.width = 100;
        map_now.info.height = 100;
        map_now.info.resolution = 0.5;
        map_now.info.origin.position.x = 0.0;
        map_now.info.origin.position.y = 0.0;
        map_now.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
        map_now.data.resize(map_now.info.width*map_now.info.height,0);

        //2.init map data
        U4 width = map_now.info.width;
        U4 height = map_now.info.height;
        std::default_random_engine e;
        e.seed(time(NULL));
        std::uniform_int_distribution<unsigned> u(0,width);
        U4 cove1,cove2;
        for(U4 y = 16;y < height;y+=18){
            cove1 = u(e);
            cove2 = u(e);
            for(U4 x = 0;x < width;x++){
                map_now.data[y*width + x] = 100;
                if(x == cove1+1 || x == cove1+2|| x == cove1+3|| x == cove1+4){
                    map_now.data[y*width + x] = 0;
                }
                if(x == cove2+1 || x == cove2+2|| x == cove2+3|| x == cove2+4){
                    map_now.data[y*width + x] = 0;
                }
            }
        }

        //3.init global planner param
        Plan_Param plan_param;
        //3.1 for all
        plan_param.plan_mode = Dijkstra;  //Dijkstra,A_Star,RPM,RRT,RRT_Star,Sweep
        plan_param.obstacle_prob = 5;      //0-100, bigger than this number is obstacle
        plan_param.reduce_scale = 1;       //Reduce resolution scale
        plan_param.use_local_plan = false; //whether use local plan
        //3.2 for Dijkstra, RPM
        //none
        //3.3 for A_Star, RPM
        plan_param.i_w = 1;                //indicator function weight
        plan_param.i_mode = manhattan;     //indicator function mode:   1:manhattan  2:Euclidean
        //3.4 kdtree config for RPM,RRT,RRT_Star,Dwa,State_lattice
        plan_param.KNsearch = false;       //true:nearest k search or false:radiusSearch
        plan_param.K = 10;                 //K in nearest k search
        plan_param.radius = 100.0;         //radius in radius search, (m)
        //3.5 bresenham line config for RPM,RRT,RRT_Star
        plan_param.obstacle_avoid = false;  //if true for nearest 9 point no block,but it will increase 9 times time
        //3.6 for RPM
        plan_param.s_mode = A_Star;        //search mode:  1:Dijkstra 2:A_Star
        plan_param.seed_num = 5000;        //random point num
        plan_param.k_choice = 9;           //search nearest k point to construct edge
        //3.7 for RRT RRT_Star
        plan_param.stop_dis = 3.5;         //stop distance to final
        plan_param.max_try_times = 500000; //max RRT times;
        plan_param.max_distance = 2.5;     //leaf max extend distance
        //3.8 for RRT_Star
        plan_param.set_dis = 10.0;          //x_rand search dis
        //3.9 for sweep , param from 3.1 to 3.8
        plan_param.block_Search = Dijkstra; //if block,to find nearest, using Dijkstra,A_Star,RRT,RRT_Star..RPM no suggest
        //3.10 for Map Refer Simulation
        plan_param.sim_wb = 2;
        plan_param.sim_v = 1;
        plan_param.sim_dt = 0.05;

        //4.init global planner pose
        Pose_6d init_pose,final_pose;
        init_pose.x = 0.25;
        init_pose.y = 0.25;
        final_pose.x = 49;
        final_pose.y = 49;

        //5.get global plan
        nav_msgs::Path path;
        R8 time = ros::Time::now().toSec();
        std::vector<Pose_6d> path_get = planner.GetPlan(map_now,init_pose,final_pose,plan_param);
        printf("\nplan cost:%f\n",ros::Time::now().toSec()-time);
        path.poses.resize(path_get.size());
        R8 yaw;

        for(U4 i = 0;i < path_get.size();i++){
            path.poses[i].pose.position.x = path_get[i].x;
            path.poses[i].pose.position.y = path_get[i].y;
            if(path_get[i+1].x == path_get[i].x){
                path_get[i].yaw = path_get[i+1].y>path_get[i].y?1.57:-1.57;
            } else {
                yaw = (path_get[i+1].y-path_get[i].y)/(path_get[i+1].x-path_get[i].x);
                yaw = atan(yaw);
                path_get[i].yaw = yaw;
            }
            path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        }

        //6.local planner init
        nav_msgs::Path local_path;
        std::vector<Pose_6d> path_local;
        //6.1 for all local planner
        plan_param.plan_mode = State_Lattice;
        plan_param.init_index = 0;
        plan_param.end_index = 50;
        //6.2 for Dwa
        plan_param.w_scale = 1;
        plan_param.w_interval = 0.1; //w interval
        plan_param.v_scale = 1;
        plan_param.v_interval = 0.1;
        plan_param.dt = 1.5;
        plan_param.min_block_dis = 1;
        plan_param.ap = 1;
        plan_param.bp = 5; //as this path
        plan_param.cp = 100;
        plan_param.out_interval = 0.01;
        //6.3 for state lattice
        plan_param.path_v = plan_param.sim_v;
        plan_param.path_dt = plan_param.sim_dt;
        plan_param.l1_scale = -0.25;
        plan_param.l1_scalep = 0.25;
        plan_param.l1_intv = 0.25;
        plan_param.s1_scale = 1;
        plan_param.s1_scalep = 10;
        plan_param.s1_intv = 1;
        plan_param.s1d_scale = 0;
        plan_param.s1d_scalep = 1;
        plan_param.s1d_intv = 0.2;
        plan_param.t1_scale = 0;
        plan_param.t1_scalep = 9;
        plan_param.t1_intv = 1;
        plan_param.sample_intv = 0.1;
        plan_param.af = 10;
        plan_param.bf = 1;
        plan_param.cf = 1;
        plan_param.out_t_intv = 0.05;
        //6.4 for Reeds shepp parking,auto car only
        plan_param.stop_pose.x = 4;       //stop pose
        plan_param.stop_pose.y = 4;       //stop pose
        plan_param.stop_pose.yaw = 7*TT/4; //stop pose 0 ~ 2*TT
        plan_param.max_search = 300;      //max search index
        plan_param.wheel_base = 2;        //wheel base /m
        plan_param.max_steer = 0.56;      //max steer angle /rad
        plan_param.t_interval = 0.05;     //t interval for detect
        plan_param.idling = -2;           //must be negative
        plan_param.yaw_bias = 0.1;       //max stop yaw bias
        plan_param.sx_bias = 1;           //max stop x bias
        plan_param.sy_bias = 0;           //max stop negative y bias
        plan_param.res_bias = 0.1;        //max stop res bias
        plan_param.out_t = 0.01;          //output t interval
        plan_param.pre_intp = false;       //use pre_interpolation
        plan_param.intp_scale = 4;        //interpolation scale no more than 8

        //6.5 init local planner pose
        geometry_msgs::PoseStamped tmp_pose;
        init_pose.x = 1;
        init_pose.y = 2;
        init_pose.yaw = 0.0;
        init_pose.v = 2;
        init_pose.w = 0;
        init_pose.a = 0;

        //7.loop
        U4 i = 0;
        while(Main_Running){
            now_time = ros::Time::now();
            map_now.header.stamp = now_time;
            map_now.header.frame_id = "/map";
            map_now.header.seq = seq;
            map_pub.publish(map_now);

            path.header.stamp = now_time;
            path.header.frame_id = "/path";
            path.header.seq = seq;
            path_pub.publish(path);

            if(plan_param.use_local_plan){
                //7.1 get local plan
                local_path.header.frame_id = seq++;
                local_path.header.stamp = now_time;
                local_path.header.frame_id = "/local_path";
                path_local.clear();
                local_path.poses.clear();
                path_local = planner.GetPlan(map_now,init_pose,plan_param,path_get);
                for(U4 i = 0;i < path_local.size();i++){
                    tmp_pose.pose.position.x = path_local[i].x;
                    tmp_pose.pose.position.y = path_local[i].y;
                    tmp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
                    local_path.poses.push_back(tmp_pose);
                }
                lpath_pub.publish(local_path);
                //7.2 update state
                if(i<path_get.size()-2 && plan_param.plan_mode != Reeds_Shepp &&
                   plan_param.plan_mode != State_Lattice){
                    init_pose.x = path_get[i].x;
                    init_pose.y = path_get[i].y;
                    init_pose.yaw = path_get[i].yaw;
                    i++;
                    plan_param.init_index = i-5;
                    if(plan_param.init_index < 0){
                        plan_param.init_index = 0;
                    }
                    plan_param.end_index = i+50;
                    if(plan_param.end_index >= path_get.size()){
                        plan_param.end_index = path_get.size() - 1;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    void MainRun::tf_publish_function(R8 time_interval){
        ros::Rate r(1.0 / time_interval);
        geometry_msgs::TransformStamped global_map_trans,local_trans;
        geometry_msgs::Quaternion th_q = tf::createQuaternionMsgFromYaw(0.0);
        while(ros::ok()){
            global_map_trans.header.stamp = ros::Time::now() + ros::Duration(time_interval);
            global_map_trans.header.seq   = tf_seq++;
            global_map_trans.header.frame_id = "/map";
            global_map_trans.child_frame_id = "/path";
            global_map_trans.transform.translation.x = 0.0; //you had known
            global_map_trans.transform.translation.y = 0.0;
            global_map_trans.transform.translation.z = 0.0;
            global_map_trans.transform.rotation = th_q;
            tf::TransformBroadcaster global_map_broadcaster;
            global_map_broadcaster.sendTransform(global_map_trans);

            local_trans.header.stamp = ros::Time::now() + ros::Duration(time_interval);
            local_trans.header.seq   = tf_seq++;
            local_trans.header.frame_id = "/map";
            local_trans.child_frame_id = "/local_path";
            local_trans.transform.translation.x = 0.0; //you had known
            local_trans.transform.translation.y = 0.0;
            local_trans.transform.translation.z = 0.0;
            local_trans.transform.rotation = th_q;
            tf::TransformBroadcaster local_broadcaster;
            local_broadcaster.sendTransform(local_trans);
            r.sleep();
        }
    }

    void MainRun::Run() {
        map_pub  = node_.advertise<nav_msgs::OccupancyGrid>("/map",1,true);
        path_pub  = node_.advertise<nav_msgs::Path>("/path",10,true);
        lpath_pub = node_.advertise<nav_msgs::Path>("/local_path",10,true);
        Main_thread = new std::thread(&MainRun::Main_Function,this);
        tf_thread = new std::thread(std::bind(&MainRun::tf_publish_function,this,0.005));
    }
}