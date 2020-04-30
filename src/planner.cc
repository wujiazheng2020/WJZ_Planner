#include "planner.h"

namespace wjz_planner {

    graph_v::graph_v(){}

    Plan_Param::Plan_Param(){
        plan_mode = Map_Refer;  //Dijkstra,A_Star,RPM,RRT,RRT_Star,Sweep
        obstacle_prob = 5;      //0-100, bigger than this number is obstacle
        reduce_scale = 1;       //Reduce resolution scale
        use_local_plan = false; //whether use local plan
        //3.2 for Dijkstra, RPM
        //none
        //3.3 for A_Star, RPM
        i_w = 1;                //indicator function weight
        i_mode = manhattan;     //indicator function mode:   1:manhattan  2:Euclidean
        //3.4 kdtree config for RPM,RRT,RRT_Star,Dwa,State_lattice
        KNsearch = false;       //true:nearest k search or false:radiusSearch
        K = 10;                 //K in nearest k search
        radius = 100.0;         //radius in radius search, (m)
        //3.5 bresenham line config for RPM,RRT,RRT_Star
        obstacle_avoid = false;  //if true for nearest 9 point no block,but it will increase 9 times time
        //3.6 for RPM
        s_mode = A_Star;        //search mode:  1:Dijkstra 2:A_Star
        seed_num = 5000;        //random point num
        k_choice = 9;           //search nearest k point to construct edge
        //3.7 for RRT RRT_Star
        stop_dis = 3.5;         //stop distance to final
        max_try_times = 500000; //max RRT times;
        max_distance = 2.5;     //leaf max extend distance
        //3.8 for RRT_Star
        set_dis = 10.0;          //x_rand search dis
        //3.9 for sweep , param from 3.1 to 3.8
        block_Search = Dijkstra; //if block,to find nearest, using Dijkstra,A_Star,RRT,RRT_Star..RPM no suggest
        //3.10 for Map Refer Simulation
        sim_wb = 2;
        sim_v = 1;
        sim_dt = 0.05;

        //6.1 for all local planner
        plan_mode = State_Lattice;
        init_index = 0;
        end_index = 50;
        //6.2 for Dwa
        w_scale = 1;
        w_interval = 0.1; //w interval
        v_scale = 1;
        v_interval = 0.1;
        dt = 1.5;
        min_block_dis = 1;
        ap = 1;
        bp = 5; //as this path
        cp = 100;
        out_interval = 0.01;
        //6.3 for state lattice
        path_v = sim_v;
        path_dt = sim_dt;
        l1_scale = -0.25;
        l1_scalep = 0.25;
        l1_intv = 0.25;
        s1_scale = 1;
        s1_scalep = 10;
        s1_intv = 1;
        s1d_scale = 0;
        s1d_scalep = 1;
        s1d_intv = 0.2;
        t1_scale = 0;
        t1_scalep = 9;
        t1_intv = 1;
        sample_intv = 0.1;
        af = 10;
        bf = 1;
        cf = 1;
        out_t_intv = 0.05;
        //6.4 for Reeds shepp parking,auto car only
        stop_pose.x = 4;       //stop pose
        stop_pose.y = 4;       //stop pose
        stop_pose.yaw = 7*TT/4; //stop pose 0 ~ 2*TT
        max_search = 300;      //max search index
        wheel_base = 2;        //wheel base /m
        max_steer = 0.56;      //max steer angle /rad
        t_interval = 0.05;     //t interval for detect
        idling = -2;           //must be negative
        yaw_bias = 0.1;       //max stop yaw bias
        sx_bias = 1;           //max stop x bias
        sy_bias = 0;           //max stop negative y bias
        res_bias = 0.1;        //max stop res bias
        out_t = 0.01;          //output t interval
        pre_intp = false;       //use pre_interpolation
        intp_scale = 4;        //interpolation scale no more than 8
    }

    Planner::Planner(){}

    Planner::~Planner(){}

    void Planner::DFS(U4 start_i,U4 end_i,U4* pre){
        index_list.push_back(start_i);
        if(pre[start_i] == end_i){
            return;
        }
        DFS(pre[start_i],end_i,pre);
    }

    R8 Planner::GetDistance(I4 x1,I4 y1,I4 x2,I4 y2,X1 mode, X1 i_w){
        if(mode == 1){
            return i_w*(abs(x1-x2) + abs(y1-y2));
        }
        if(mode == 2){
            return i_w*sqrt((x1-x2)*(x1-x2) + abs(y1-y2)*(y1-y2));
        }
        return 0;
    }

    void Planner::Reduce_Resolution(nav_msgs::OccupancyGrid &map_before,nav_msgs::OccupancyGrid &map_now,U4 scale){

        map_now.info.width  = map_before.info.width/scale;
        map_now.info.height = map_before.info.height/scale;
        map_now.info.resolution = map_before.info.resolution*scale;
        map_now.info.origin = map_before.info.origin;
        map_now.header = map_before.header;
        map_now.data.resize(map_now.info.width*map_now.info.height,0);
        U4 width = map_before.info.width;
        U4 height = map_before.info.height;
        U4 width_n = map_now.info.width;
        I4 now_x,now_y;
        for(I4 y=0;y<height;y++){
            for(I4 x=0;x<width;x++){
                if(map_before.data[y*width + x]<0 || map_before.data[y*width + x]>5){
                    now_x = x/scale;
                    now_y = y/scale;
                    map_now.data[now_y*width_n + now_x] = 100;
                }
            }
        }
    }

    std::vector<Pose_6d> Planner::Linear_Interpolation(std::vector<Pose_6d> &refer_line,U4 scale){
        std::vector<Pose_6d> refer_now;
        Pose_6d tmp_pose;
        R8 k,tmp;
        if(scale == 1){
            return refer_line;
        }
        for(U4 i = 0;i<refer_line.size()-1;i++){
            k = tan(refer_line[i].yaw);
            refer_now.push_back(refer_line[i]);
            for(U4 j = 1;j<scale;j++){
                tmp = (refer_line[i+1].x - refer_line[i].x)/scale*j;
                tmp_pose.x = refer_line[i].x + tmp;
                tmp_pose.y = refer_line[i].y + tmp*k;
                tmp_pose.yaw = refer_line[i].yaw;
                refer_now.push_back(tmp_pose);
            }
        }
        return refer_now;
    }

    R8 Planner::yaw_sub(R8 yaw1,R8 yaw2){
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
        if(dyaw < -TT){
            dyaw += 2*TT;
        } else if(dyaw > TT){
            dyaw -= 2*TT;
        }
        return dyaw;
    }

    R8 Planner::yaw_add(R8 yaw1,R8 yaw2){
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

    std::vector<Pose_6d> Planner::Cubic_Spline(std::vector<Pose_6d> &refer_line){
//        tk::spline sx,sy;
//        std::vector<R8> t;
//        std::vector<R8> x;
//        std::vector<R8> y;
//        for(I4 i = 0;i<refer_line.size();i++){
//            t.push_back(i);
//            x.push_back(refer_line[i].x);
//            y.push_back(refer_line[i].y);
//        }
//        sx.set_points(t,x);
//        sy.set_points(t,y);
//
        std::vector<Pose_6d> path_get;
//        Pose_6d tmp_pose;
//        for(R8 t1 = 0;t1<refer_line.size();t1+=0.01){
//            tmp_pose.x = sx(t1);
//            tmp_pose.y = sy(t1);
//            path_get.push_back(tmp_pose);
//        }
        return path_get;
    }

    std::vector<Pose_6d> Planner::GetPlan(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                          Pose_6d &final_pose,Plan_Param &param){
        nav_msgs::OccupancyGrid map_now;
        if(param.reduce_scale >= 2){
            Reduce_Resolution(map,map_now,param.reduce_scale);
        } else {
            map_now = map;
        }

        switch (param.plan_mode) {
            case Dijkstra:
                return Dijkstra_2D(map_now,init_pose,final_pose,param);
                break;
            case A_Star:
                return A_Star_2D(map_now,init_pose,final_pose,param);
                break;
            case RPM:
                return RPM_2D(map_now,init_pose,final_pose,param);
                break;
            case RRT:
                return RRT_2D(map_now,init_pose,final_pose,param);
                break;
            case RRT_Star:
                return RRT_Star_2D(map_now,init_pose,final_pose,param);
                break;
            case Sweep:
                return Sweep_2D(map_now,init_pose,final_pose,param);
                break;
            case Map_Refer:
                return Map_Refer_Simulation(init_pose,param);
            default:
                printf("Plan Mode Error,Please check plan mode!");
                break;
        }

    }

    std::vector<Pose_6d> Planner::GetPlan(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                          Plan_Param &param,std::vector<Pose_6d> &refer_line){
        switch (param.plan_mode) {
            case Dwa:
                return Dwa_2D(map,init_pose,param,refer_line);
                break;
            case State_Lattice:
                return State_Lattice_2D(map,init_pose,param,refer_line);
                break;
            case Reeds_Shepp:
                return Reeds_Shepp_2D(map,init_pose,param,refer_line);
                break;
            default:
                printf("Plan Mode Error,Please check plan mode!");
                break;
        }
    }

    std::vector<Pose_6d> Planner::Dijkstra_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                              Pose_6d &final_pose,Plan_Param &param){
        R8 ox     = map.info.origin.position.x;
        R8 oy     = map.info.origin.position.y;
        R8 rslo   = map.info.resolution;
        U4 width  = map.info.width;
        U4 height = map.info.height;

        I4 init_gx  = floor((init_pose.x - ox)/rslo);
        I4 init_gy  = floor((init_pose.y - oy)/rslo);
        I4 final_gx = floor((final_pose.x - ox)/rslo);
        I4 final_gy = floor((final_pose.y - oy)/rslo);
        I4 init_j   = init_gy*width + init_gx;
        I4 final_j  = final_gy*width + final_gx;
        if(map.data[final_j]<0 || map.data[final_j]>param.obstacle_prob){
            assert(false);
        }
        if(final_gx<0 || final_gx>=width || final_gy<0 || final_gy>=height){
            assert(false);
        }

        //1.init cost matrix,obstacle give -1,pre matrix
        index_list.clear();
        U4 map_size = map.data.size();
        U4 pre[map_size];
        R8 cost[map_size];
        R8 max_cost = 1e10;
        for(U4 i = 0;i < map_size;i++){
            pre[i] = init_j;
            if(map.data[i]<0 || map.data[i]>param.obstacle_prob){
                cost[i] = -1;
            }else {
                cost[i] = max_cost;
            }
        }
        cost[init_j] = -1;

        //2.init nearest cost
        I4 now_i;
        X1 index_dis;
        for(I4 y = init_gy-1;y <= init_gy+1;y++){
            for(I4 x = init_gx-1;x <= init_gx+1;x++){
                now_i = y*width + x;
                if(x<0 || x>=width || y<0 || y>=height){
                    continue;
                }
                if(cost[now_i] != -1 && now_i != init_j){
                    index_dis = abs(x-init_gx) + abs(y-init_gy);
                    if(index_dis == 1){
                        cost[now_i] = 1;
                    }
                    if(index_dis == 2){
                        cost[now_i] = 1.41421356;
                    }
                }
            }
        }

        //3.iter
        I4 now_x,now_y;
        I4 tmp_i,min_i=0;
        R8 min_dis;
        R8 graph_jk=0;
        B1 over = false;
        now_i = init_j;
        std::list<U4>::iterator index_iter;
        std::list<U4>::iterator min_iter;
        while(now_i != final_j){
            //3.1 find min index
            over = true;
            min_dis = max_cost;
            for(tmp_i = 0;tmp_i<map_size;tmp_i++){
                if(cost[tmp_i] < min_dis && cost[tmp_i] != -1){
                    min_i = tmp_i;
                    min_dis = cost[tmp_i];
                    over = false;
                }
            }
            if(over || min_i == final_j){break;}

            //3.2 update cost
            now_x = min_i%width;
            now_y = min_i/width;
            for(I4 y = now_y-1;y <= now_y+1;y++){
                for(I4 x = now_x-1;x <= now_x+1;x++){
                    tmp_i = y*width + x;
                    if(x<0 || x>=width || y<0 || y>=height || tmp_i == min_i || cost[tmp_i] == -1){
                        continue;
                    }
                    index_dis = abs(x-now_x) + abs(y-now_y);
                    graph_jk = (index_dis == 1)?1:1.41421356;
                    if(cost[tmp_i] > cost[min_i]+graph_jk){
                        cost[tmp_i] = cost[min_i]+graph_jk;
                        pre[tmp_i] = min_i;
                    }
                }
            }

            //3.3 update now state
            now_i = min_i;
            cost[now_i] = -1;
        }

        //4.get path
        std::vector<Pose_6d> path_get;
        if(over){
            printf("cannot find path");
            path_get.push_back(init_pose);
            return path_get;
        }
        DFS(final_j,init_j,pre);
        Pose_6d tmp_pose;
        U4 path_size = index_list.size();
        tmp_pose.x = init_pose.x;
        tmp_pose.y = init_pose.y;
        path_get.push_back(tmp_pose);
        for(I4 i = path_size-1;i >= 1;i--){
            tmp_pose.x = (index_list[i]%width)*rslo + ox + rslo/2;
            tmp_pose.y = (index_list[i]/width)*rslo + oy + rslo/2;
            path_get.push_back(tmp_pose);
        }
        tmp_pose.x = final_pose.x;
        tmp_pose.y = final_pose.y;
        path_get.push_back(tmp_pose);
        return path_get;
    }

    std::vector<Pose_6d> Planner::A_Star_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                            Pose_6d &final_pose,Plan_Param &param){
        R8 ox     = map.info.origin.position.x;
        R8 oy     = map.info.origin.position.y;
        R8 rslo   = map.info.resolution;
        U4 width  = map.info.width;
        U4 height = map.info.height;

        I4 init_gx  = floor((init_pose.x - ox)/rslo);
        I4 init_gy  = floor((init_pose.y - oy)/rslo);
        I4 final_gx = floor((final_pose.x - ox)/rslo);
        I4 final_gy = floor((final_pose.y - oy)/rslo);
        I4 init_j   = init_gy*width + init_gx;
        I4 final_j  = final_gy*width + final_gx;
        if(map.data[final_j]<0 || map.data[final_j]>param.obstacle_prob){
            assert(false);
        }
        if(final_gx<0 || final_gx>=width || final_gy<0 || final_gy>=height){
            assert(false);
        }

        //1.init cost matrix,obstacle give -1,pre matrix,distance matrix
        index_list.clear();
        I4 tmp_x,tmp_y;
        U4 map_size = map.data.size();
        U4 pre[map_size];
        R8 cost[map_size];
        R8 dis_cost[map_size];
        R8 max_cost = 1e10;
        for(U4 i = 0;i < map_size;i++){
            tmp_x = i%width;
            tmp_y = i/width;
            dis_cost[i] = GetDistance(final_gx,final_gy,tmp_x,tmp_y,param.i_mode,param.i_w);
            pre[i] = init_j;
            if(map.data[i]<0 || map.data[i]>param.obstacle_prob){
                cost[i] = -1;
            }else {
                cost[i] = max_cost;
            }
        }
        cost[init_j] = -1;

        //2.init nearest cost
        I4 now_i;
        X1 index_dis;
        for(I4 y = init_gy-1;y <= init_gy+1;y++){
            for(I4 x = init_gx-1;x <= init_gx+1;x++){
                now_i = y*width + x;
                if(x<0 || x>=width || y<0 || y>=height){
                    continue;
                }
                if(cost[now_i] != -1 && now_i != init_j){
                    index_dis = abs(x-init_gx) + abs(y-init_gy);
                    if(index_dis == 1){
                        cost[now_i] = 1;
                    }
                    if(index_dis == 2){
                        cost[now_i] = 1.41421356;
                    }
                }
            }
        }

        //3.iter
        I4 now_x,now_y;
        I4 tmp_i,min_i=0;
        R8 min_dis;
        R8 mix_dis;
        R8 graph_jk=0;
        B1 over = false;
        now_i = init_j;
        while(now_i != final_j){
            //3.1 find min index
            over = true;
            min_dis = max_cost;
            for(tmp_i = 0;tmp_i<map_size;tmp_i++){
                mix_dis = cost[tmp_i] + dis_cost[tmp_i];
                if(mix_dis < min_dis && cost[tmp_i] != -1){
                    min_i = tmp_i;
                    min_dis = mix_dis;
                    over = false;
                }
            }
            if(over || min_i == final_j){break;}

            //3.2 update cost
            now_x = min_i%width;
            now_y = min_i/width;
            for(I4 y = now_y-1;y <= now_y+1;y++){
                for(I4 x = now_x-1;x <= now_x+1;x++){
                    tmp_i = y*width + x;
                    if(x<0 || x>=width || y<0 || y>=height || tmp_i==min_i || cost[tmp_i] == -1){
                        continue;
                    }
                    index_dis = abs(x-now_x) + abs(y-now_y);
                    graph_jk = (index_dis == 1)?1:1.41421356;
                    if(cost[tmp_i] > cost[min_i]+graph_jk){
                        cost[tmp_i] = cost[min_i]+graph_jk;
                        pre[tmp_i] = min_i;
                    }
                }
            }

            //3.3 update now state
            now_i = min_i;
            cost[now_i] = -1;
        }

        //4.get path
        std::vector<Pose_6d> path_get;
        if(over){
            printf("cannot find path");
            path_get.push_back(init_pose);
            return path_get;
        }
        DFS(final_j,init_j,pre);
        Pose_6d tmp_pose;
        U4 path_size = index_list.size();
        tmp_pose.x = init_pose.x;
        tmp_pose.y = init_pose.y;
        path_get.push_back(tmp_pose);
        for(I4 i = path_size-1;i >= 1;i--){
            tmp_pose.x = (index_list[i]%width)*rslo + ox + rslo/2;
            tmp_pose.y = (index_list[i]/width)*rslo + oy + rslo/2;
            path_get.push_back(tmp_pose);
        }
        tmp_pose.x = final_pose.x;
        tmp_pose.y = final_pose.y;
        path_get.push_back(tmp_pose);
        return path_get;
    }

    std::vector<Pose_6d> Planner::RPM_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                         Pose_6d &final_pose,Plan_Param &param){
        R8 ox     = map.info.origin.position.x;
        R8 oy     = map.info.origin.position.y;
        R8 rslo   = map.info.resolution;
        U4 width  = map.info.width;
        U4 height = map.info.height;

        I4 init_gx  = floor((init_pose.x - ox)/rslo);
        I4 init_gy  = floor((init_pose.y - oy)/rslo);
        I4 final_gx = floor((final_pose.x - ox)/rslo);
        I4 final_gy = floor((final_pose.y - oy)/rslo);
        I4 init_j   = init_gy*width + init_gx;
        I4 final_j  = final_gy*width + final_gx;
        if(map.data[final_j]<0 || map.data[final_j]>param.obstacle_prob){
            assert(false);
        }
        if(final_gx<0 || final_gx>=width || final_gy<0 || final_gy>=height){
            assert(false);
        }

        //1.init pre matrix,it contains available id
        index_list.clear();
        U4 map_size = map.data.size();
        R8 max_cost = 1e10;
        std::vector<U4> pre_list;  //pre choose list
        for(U4 i = 0;i < map_size;i++){
            if(map.data[i]>=0 && map.data[i]<=param.obstacle_prob){
                if(i != init_j && i != final_j){
                    pre_list.push_back(i);
                }
            }
        }

        //2.init random generator
        U4 pre_size = pre_list.size();
        std::default_random_engine e;
        e.seed(time(NULL));
        std::uniform_int_distribution<unsigned> u(0,pre_size-1);

        //3.insert sort to erase duplicate point
        I4 random_i,now_i;
        std::list<U4> random_list;
        random_i = pre_list[u(e)];  //real id
        random_list.push_back(random_i);
        std::list<U4>::iterator random_iter;
        for(U4 i = 0;i<param.seed_num;i++){
            //3.1 get random_i.judge head first
            random_i = pre_list[u(e)];
            random_iter = random_list.begin();
            if(random_i < *random_iter){
                random_list.push_front(random_i);
                continue;
            }
            if(random_i == *random_iter){
                continue;
            }

            //3.2 judge tail second
            random_iter = random_list.end();
            random_iter--;
            if(random_i > *random_iter){
                random_list.push_back(random_i);
                continue;
            }
            if(random_i == *random_iter){
                continue;
            }

            //3.3 judge body then
            for(;random_iter!=random_list.begin();random_iter--){
                now_i = *random_iter;
                if(random_i > now_i){
                    random_list.insert(++random_iter,random_i);
                    break;
                }
                if(random_i == now_i){
                    break;
                }
            }
        }
        random_list.push_back(init_j);
        random_list.push_back(final_j);//push init and end

        //4.make kd tree,vector
        U4 g_size = random_list.size();
        U4 back_arr[map_size];  //back pointer
        I4 point_arr[g_size];
        pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
        pcl::PointXY tmp{0,0};
        cloud->points.resize(g_size);
        U4 i = 0;
        for(random_iter = random_list.begin();random_iter != random_list.end();random_iter++){
            now_i = *random_iter;
            tmp.x = now_i%width;
            tmp.y = now_i/width;
            cloud->points[i] = tmp;//f to real
            point_arr[i] = now_i; //f to real
            back_arr[now_i] = i; //for back search  real to f
            i++;
        }
        cloud->height = 1;
        cloud->width  = g_size;
        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        kdtree.setInputCloud(cloud);

        //5.construct graph,init graph
        std::vector<int> pointIdSearch;
        std::vector<float> pointSquaredDistance;
        R4 radius = ceil(param.radius/rslo);
        pcl::PointXY searchPoint{0,0};
        I4 x1,y1,x0,y0;
        I4 x0_r,y0_r,x1_r,y1_r;
        R8 k;
        B1 kover1,swap_x;
        B1 no_obs;
        I4 tp;
        I4 dx,dy,d_err,err,y_step,y;
        I4 tmp_i;
        I4 i_e; //end;

        //5.1 init graph
        graph_v graph[g_size];
        U4 n = 0;
        for(i = 0;i<g_size;i++){
            //5.2 search K nearest point;
            searchPoint = cloud->points[i];
            if(param.KNsearch){
                if(kdtree.nearestKSearch (searchPoint, param.K, pointIdSearch, pointSquaredDistance)>0){}
            }
            if(!param.KNsearch){
                if(kdtree.radiusSearch (searchPoint, radius, pointIdSearch, pointSquaredDistance)>0){}
            }
            //5.3 for each point,use bresenham line to detect obstacle
            x0_r = searchPoint.x;
            y0_r = searchPoint.y;
            for(U4 j = 1;j<=param.k_choice;j++){//no include itself
                x0 = x0_r;
                y0 = y0_r;
                x1 = cloud->points[ pointIdSearch[j] ].x;//r
                y1 = cloud->points[ pointIdSearch[j] ].y;
                x1_r = x1;
                y1_r = y1;
                i_e = y1*width + x1; //r

                if(x1 == x0){
                    k = 100;//avoid bug
                }else{
                    k = (R8)(y1-y0)/(x1-x0);
                }
                kover1 = (fabs(k)>1);
                swap_x = false;

                if(kover1){  //big slope ,swap xy
                    tp = x0; //tp:tmp
                    x0 = y0;
                    y0 = tp;

                    tp = x1;
                    x1 = y1;
                    y1 = tp;
                }
                if(x0 > x1){  //swap x0 x1
                    tp = x0;
                    x0 = x1;
                    x1 = tp;

                    tp = y0;
                    y0 = y1;
                    y1 = tp;
                    swap_x = true;
                }

                dx = x1 - x0;
                dy = y1 - y0;
                d_err = 2 * abs(dy);
                err = 0;
                y = y0;
                y_step = y0 < y1? 1:-1;

                //5.4 detect obstacle
                no_obs = true;
                for(I4 xi = x0;xi <= x1;xi++){
                    if(!param.obstacle_avoid){
                        if(kover1){
                            tmp_i = xi*width + y;
                        }else{
                            tmp_i = y*width + xi;
                        }
                        if(map.data[tmp_i] < 0 || map.data[tmp_i] > param.obstacle_prob){
                            no_obs = false;
                        }
                    } else {
                        for(I4 xa = xi-1;xa<=xi+1;xa++){
                            for(I4 ya = y-1;ya<=y+1;ya++){
                                if (kover1) {
                                    if(ya >= width || ya < 0 || xa >= height || xa < 0){
                                        continue;
                                    }
                                    tmp_i = xa * width + ya;
                                } else {
                                    if(xa >= width || xa < 0 || ya >= height || ya < 0){
                                        continue;
                                    }
                                    tmp_i = ya * width + xa;
                                }
                                if(map.data[tmp_i] < 0 || map.data[tmp_i] > param.obstacle_prob){
                                    no_obs = false;
                                }
                            }
                        }
                    }
                    if(!no_obs){
                        break;
                    }
                    err += d_err;
                    if(err > dx){
                        y += y_step;
                        err -= (2*dx);
                    }
                }

                //5.5 if no obstacle, count graph
                if(no_obs){
                    graph[i].corr_index.push_back(back_arr[i_e]);
                    graph[i].corr_dis.push_back(sqrt((x1_r-x0_r)*(x1_r-x0_r) + (y1_r-y0_r)*(y1_r-y0_r)));
                }
            }
            pointIdSearch.clear();
            pointSquaredDistance.clear();
        }

        //6.init cost matrix, for i in graph and cost ,you must map it to map point;
        R8 cost[g_size];
        R8 dis_cost[g_size];
        U4 pre[g_size];
        I4 xt,yt;
        for(i = 0;i<g_size;i++){
            pre[i] = back_arr[init_j];
            cost[i] = max_cost;
            xt = point_arr[i]%width;
            yt = point_arr[i]/width;
            dis_cost[i] = GetDistance(final_gx,final_gy,xt,yt,param.i_mode,param.i_w);
        }
        U4 i_size = graph[back_arr[init_j]].corr_index.size();
        U4 index;
        R8 dist;
        for(i = 0;i<i_size;i++){
            index = graph[back_arr[init_j]].corr_index[i]; //f
            dist = graph[back_arr[init_j]].corr_dis[i];
            cost[index] = dist;
        }
        cost[back_arr[init_j]] = -1;

        //7.use Dijkstra or A_Star to find a path;
        U4 tmp_index;
        I4 min_i=0;
        R8 min_dis;
        R8 mix_dis=0;
        B1 over = false;
        now_i = back_arr[init_j];
        while(now_i != back_arr[final_j]){ //f
            //7. find min index
            over = true;
            min_dis = max_cost;
            for(tmp_i = 0;tmp_i<g_size;tmp_i++){
                mix_dis = cost[tmp_i];
                if(param.s_mode == A_Star && min_dis != -1){
                    mix_dis += dis_cost[tmp_i];
                }
                if(mix_dis < min_dis && cost[tmp_i] != -1){
                    min_i = tmp_i;
                    min_dis = mix_dis;
                    over = false;
                }
            }
            if(over || min_i == back_arr[final_j] ){break;}

            //7.2 update cost
            index = graph[min_i].corr_index.size();
            for(tmp_i = 0;tmp_i<index;tmp_i++){
                tmp_index = graph[min_i].corr_index[tmp_i];
                if(cost[tmp_index] == -1){
                    continue;
                }
                dist = graph[min_i].corr_dis[tmp_i];
                if(cost[tmp_index] > cost[min_i] + dist){
                    cost[tmp_index] = cost[min_i] + dist;
                    pre[tmp_index] = min_i;
                }
            }

            now_i = min_i;
            cost[now_i] = -1;
        }

        //8.get path
        std::vector<Pose_6d> path_get;
        if(over){
            printf("cannot find path");
            path_get.push_back(init_pose);
            return path_get;
        }
        DFS(back_arr[final_j],back_arr[init_j],pre);
        Pose_6d tmp_pose;
        U4 path_size = index_list.size();
        tmp_pose.x = init_pose.x;
        tmp_pose.y = init_pose.y;
        path_get.push_back(tmp_pose);
        for(i = path_size-1;i >= 1;i--){
            tmp_pose.x = (point_arr[index_list[i]]%width)*rslo + ox + rslo/2;
            tmp_pose.y = (point_arr[index_list[i]]/width)*rslo + oy + rslo/2;
            path_get.push_back(tmp_pose);
        }
        tmp_pose.x = final_pose.x;
        tmp_pose.y = final_pose.y;
        path_get.push_back(tmp_pose);
        return path_get;
    }

    std::vector<Pose_6d> Planner::RRT_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                         Pose_6d &final_pose,Plan_Param &param){
        R8 ox     = map.info.origin.position.x;
        R8 oy     = map.info.origin.position.y;
        R8 rslo   = map.info.resolution;
        U4 width  = map.info.width;
        U4 height = map.info.height;

        I4 init_gx  = floor((init_pose.x - ox)/rslo);
        I4 init_gy  = floor((init_pose.y - oy)/rslo);
        I4 final_gx = floor((final_pose.x - ox)/rslo);
        I4 final_gy = floor((final_pose.y - oy)/rslo);
        I4 init_j   = init_gy*width + init_gx;
        I4 final_j  = final_gy*width + final_gx;
        if(map.data[final_j]<0 || map.data[final_j]>param.obstacle_prob){
            assert(false);
        }
        if(final_gx<0 || final_gx>=width || final_gy<0 || final_gy>=height){
            assert(false);
        }

        //1.init pre matrix,it contains available id,and pre pointer arr
        index_list.clear();
        U4 map_size = map.data.size();
        U4 pre[map_size];
        std::vector<U4> pre_list;  //pre choose list
        for(U4 i = 0;i < map_size;i++){
            pre[i] = init_j;
            if(map.data[i]>=0 && map.data[i]<=param.obstacle_prob){
                if(i != init_j && i != final_j){
                    pre_list.push_back(i);
                }
            }
        }

        //2.init random generator
        U4 pre_size = pre_list.size();
        std::default_random_engine e;
        e.seed(time(NULL));
        std::uniform_int_distribution<unsigned> u(0,pre_size-1);

        //3.use bresenham to extend tree
        U4 random_id=0,tmp_i,now_i;
        R8 dis = 100;
        I4 x0,y0,x1,y1,tp;
        R8 k;
        B1 kover1,swap_x;
        I4 dx,dy,d_err,err,y_step,y;

        //3.1 init kd tree
        pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
        pcl::PointXY tmp{(R4)init_gx,(R4)init_gy};
        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        cloud->points.push_back(tmp);
        cloud->height = 1;
        cloud->width  = 1;
        kdtree.setInputCloud(cloud);
        pcl::PointXY searchPoint{0,0};
        std::vector<int> pointIdSearch;
        std::vector<float> pointSquaredDistance;
        R4 radius = ceil(param.radius/rslo);
        B1 no_obs;
        U4 try_times=0;
        R8 stop_dis = floor(param.stop_dis/rslo);
        R8 max_dis = param.max_distance/rslo;
        U4 x_max,y_max;
        R8 theta=0;
        U4 search_i=0;
        U4 end_i=0;

        while(dis >= stop_dis*stop_dis && try_times < param.max_try_times){
            //3.1 get random id
            random_id = pre_list[u(e)];

            //3.2 find nearest point in three;
            searchPoint.x = (R8)(random_id%width);
            searchPoint.y = (R8)(random_id/width);
            x1 = searchPoint.x;
            y1 = searchPoint.y;
            if(param.KNsearch){
                if(kdtree.nearestKSearch (searchPoint, param.K, pointIdSearch, pointSquaredDistance)>0){}
            }
            if(!param.KNsearch){
                if(kdtree.radiusSearch (searchPoint, radius, pointIdSearch, pointSquaredDistance)>0){}
            }
            x0 = cloud->points[ pointIdSearch[0] ].x;//r
            y0 = cloud->points[ pointIdSearch[0] ].y;
            now_i = y0*width + x0;

            if(x1 == x0){
                if(y1>y0){
                    theta = 1.5707963;
                }
                if(y1<y0){
                    theta = -1.5707963;
                }
            }else{
                theta = get_angle(y1,x1,y0,x0);
            }
            swap_x = false;

            //3.3 get max x1,x2
            x_max = x0 + floor(max_dis*cos(theta));
            y_max = y0 + floor(max_dis*sin(theta));
            if(x1>x_max && x_max>=x0 && y1>y_max && y_max>=y0){
                x1 = x_max;
                y1 = y_max;
            }
            if(x1<x_max && x_max<=x0 && y1<y_max && y_max<=y0){
                x1 = x_max;
                y1 = y_max;
            }

            if(x0 == x1 && y0 == y1){
                continue;
            }
            search_i = y1*width + x1;
            if(pre[search_i] != init_j){
                continue;
            }

            if(x1 == x0){
                kover1 = true;
            }else{
                kover1 = (fabs((R8)(y1-y0)/(x1-x0))>1);
            }

            if(kover1){  //big slope ,swap xy
                tp = x0; //tp:tmp
                x0 = y0;
                y0 = tp;

                tp = x1;
                x1 = y1;
                y1 = tp;
            }
            if(x0 > x1){  //swap x0 x1
                tp = x0;
                x0 = x1;
                x1 = tp;

                tp = y0;
                y0 = y1;
                y1 = tp;
                swap_x = true;
            }

            dx = x1 - x0;
            dy = y1 - y0;
            d_err = 2 * abs(dy);
            err = 0;
            y = y0;
            y_step = y0 < y1? 1:-1;

            //3.4 detect obstacle
            no_obs = true;
            for(I4 xi = x0;xi <= x1;xi++){
                if(!param.obstacle_avoid){
                    if(kover1){
                        tmp_i = xi*width + y;
                    }else{
                        tmp_i = y*width + xi;
                    }
                    if(map.data[tmp_i] < 0 || map.data[tmp_i] > param.obstacle_prob){
                        no_obs = false;
                    }
                } else {
                    for (I4 xa = xi - 1; xa <= xi + 1; xa++) {
                        for (I4 ya = y - 1; ya <= y + 1; ya++) {
                            if (kover1) {
                                if(ya >= width || ya < 0 || xa >= height || xa < 0){
                                    continue;
                                }
                                tmp_i = xa * width + ya;
                            } else {
                                if(xa >= width || xa < 0 || ya >= height || ya < 0){
                                    continue;
                                }
                                tmp_i = ya * width + xa;
                            }
                            if (map.data[tmp_i] < 0 || map.data[tmp_i] > param.obstacle_prob) {
                                no_obs = false;
                            }
                        }
                    }
                }
                if(!no_obs){
                    break;
                }
                err += d_err;
                if(err > dx){
                    y += y_step;
                    err -= (2*dx);
                }
            }

            //3.5 if no obs ,update three
            if(no_obs){
                pre[search_i] = now_i;
                end_i = search_i;
                searchPoint.x = (R8)(search_i%width);
                searchPoint.y = (R8)(search_i/width);
                cloud->points.push_back(searchPoint);
                kdtree.setInputCloud(cloud);
                dis = (searchPoint.x - final_gx)*(searchPoint.x - final_gx) +
                      (searchPoint.y - final_gy)*(searchPoint.y - final_gy) ;
            }
            try_times++;
            pointIdSearch.clear();
            pointSquaredDistance.clear();
        }

        //4.get path
        std::vector<Pose_6d> path_get;
        if(try_times >= param.max_try_times){
            printf("cannot find path");
            path_get.push_back(init_pose);
            return path_get;
        }
        DFS(end_i,init_j,pre);
        Pose_6d tmp_pose;
        U4 path_size = index_list.size();
        tmp_pose.x = init_pose.x;
        tmp_pose.y = init_pose.y;
        path_get.push_back(tmp_pose);
        for(I4 i = path_size-1;i >= 0;i--){
            tmp_pose.x = (index_list[i]%width)*rslo + ox + rslo/2;
            tmp_pose.y = (index_list[i]/width)*rslo + oy + rslo/2;
            path_get.push_back(tmp_pose);
        }
        tmp_pose.x = final_pose.x;
        tmp_pose.y = final_pose.y;
        path_get.push_back(tmp_pose);
        return path_get;
    }

    std::vector<Pose_6d> Planner::RRT_Star_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                              Pose_6d &final_pose,Plan_Param &param){
        R8 ox     = map.info.origin.position.x;
        R8 oy     = map.info.origin.position.y;
        R8 rslo   = map.info.resolution;
        U4 width  = map.info.width;
        U4 height = map.info.height;

        I4 init_gx  = floor((init_pose.x - ox)/rslo);
        I4 init_gy  = floor((init_pose.y - oy)/rslo);
        I4 final_gx = floor((final_pose.x - ox)/rslo);
        I4 final_gy = floor((final_pose.y - oy)/rslo);
        I4 init_j   = init_gy*width + init_gx;
        I4 final_j  = final_gy*width + final_gx;
        if(map.data[final_j]<0 || map.data[final_j]>param.obstacle_prob){
            assert(false);
        }
        if(final_gx<0 || final_gx>=width || final_gy<0 || final_gy>=height){
            assert(false);
        }

        //1.init pre matrix,it contains available id,and pre pointer arr
        index_list.clear();
        U4 map_size = map.data.size();
        U4 pre[map_size];
        std::vector<U4> pre_list;  //pre choose list
        for(U4 i = 0;i < map_size;i++){
            pre[i] = init_j;
            if(map.data[i]>=0 && map.data[i]<=param.obstacle_prob){
                if(i != init_j && i != final_j){
                    pre_list.push_back(i);
                }
            }
        }

        //2.init random generator
        U4 pre_size = pre_list.size();
        std::default_random_engine e;
        e.seed(time(NULL));
        std::uniform_int_distribution<unsigned> u(0,pre_size-1);

        //3.use bresenham to extend tree
        U4 random_id=0,tmp_i;
        R8 dis = 10000;
        I4 x0,y0,x1,y1,tp;
        R8 k;
        B1 kover1,swap_x;
        I4 dx,dy,d_err,err,y_step,y;

        //3.1 init kd tree
        pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
        pcl::PointXY tmp{(R4)init_gx,(R4)init_gy};
        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        cloud->points.push_back(tmp);
        cloud->height = 1;
        cloud->width  = 1;
        kdtree.setInputCloud(cloud);
        pcl::PointXY searchPoint{0,0};
        std::vector<int> pointIdSearch;
        std::vector<float> pointSquaredDistance;
        R4 radius = ceil(param.radius/rslo);
        B1 no_obs;
        U4 try_times=0;
        R8 set_dis = param.set_dis/rslo;
        R8 stop_dis = floor(param.stop_dis/rslo);
        R8 max_dis = param.max_distance/rslo;
        U4 x_max,y_max;
        R8 theta=0;
        I4 search_i=0;
        I4 end_i=0;
        I4 x1_r,y1_r;
        I4 now_k;
        R8 min_dis;
        I4 min_i=0;
        I4 node_i;
        B1 has_one;

        while(dis >= stop_dis*stop_dis && try_times < param.max_try_times){
            //3.1 get random id
            random_id = pre_list[u(e)];

            //3.2 find nearest point in three;
            searchPoint.x = (R8)(random_id%width);
            searchPoint.y = (R8)(random_id/width);
            x1 = searchPoint.x;
            y1 = searchPoint.y;
            if(param.KNsearch){
                if(kdtree.nearestKSearch (searchPoint, param.K, pointIdSearch, pointSquaredDistance)>0){}
            }
            if(!param.KNsearch){
                if(kdtree.radiusSearch (searchPoint, radius, pointIdSearch, pointSquaredDistance)>0){}
            }
            x0 = cloud->points[ pointIdSearch[0] ].x;//r
            y0 = cloud->points[ pointIdSearch[0] ].y;
            if(x0 == x1 && y0 == y1){
                continue;
            }

            if(x1 == x0){
                if(y1>y0){
                    theta = 1.5707963;
                }
                if(y1<y0){
                    theta = -1.5707963;
                }
            }else{
                theta = get_angle(y1,x1,y0,x0);
            }
            swap_x = false;

            //3.3 get max x1,x2
            x_max = x0 + floor(max_dis*cos(theta));
            y_max = y0 + floor(max_dis*sin(theta));
            B1 cut = false;
            if(x1>x_max && x_max>=x0 && y1>y_max && y_max>=y0){
                x1 = x_max;
                y1 = y_max;
                cut = true;
            }
            if(x1<x_max && x_max<=x0 && y1<y_max && y_max<=y0){
                x1 = x_max;
                y1 = y_max;
                cut = true;
            }

            search_i = y1*width + x1;
            if(pre[search_i] != init_j){
                continue;
            }
            x1_r = x1;
            y1_r = y1;
            //3.3.1 if cut,search again;
            if(cut){
                searchPoint.x = (R8)x1;
                searchPoint.y = (R8)y1;
                pointIdSearch.clear();
                pointSquaredDistance.clear();
                if(param.KNsearch){
                    if(kdtree.nearestKSearch (searchPoint, param.K, pointIdSearch, pointSquaredDistance)>0){}
                }
                if(!param.KNsearch){
                    if(kdtree.radiusSearch (searchPoint, radius, pointIdSearch, pointSquaredDistance)>0){}
                }
            }

            //3.4 for each point,iter for distance
            R8 g_dis=0;
            now_k = 0;
            min_dis = 999999999;
            has_one = false;
            while(now_k < pointIdSearch.size()){
                x0 = cloud->points[pointIdSearch[now_k]].x;
                y0 = cloud->points[pointIdSearch[now_k]].y;
                x1 = x1_r;
                y1 = y1_r;
                if(x0 == x1 && y0 == y1){
                    now_k ++;
                    continue;
                }
                g_dis = (x1-x0)*(x1-x0) + (y1-y0)*(y1-y0);
                if(g_dis > set_dis*set_dis && now_k >=1){
                    break;
                }
                node_i = y0*width + x0;//x0 y0
                if(x1 == x0){
                    kover1 = true;
                }else{
                    kover1 = (fabs((R8)(y1-y0)/(x1-x0))>1);
                }

                if(kover1){  //big slope ,swap xy
                    tp = x0; //tp:tmp
                    x0 = y0;
                    y0 = tp;

                    tp = x1;
                    x1 = y1;
                    y1 = tp;
                }
                if(x0 > x1){  //swap x0 x1
                    tp = x0;
                    x0 = x1;
                    x1 = tp;

                    tp = y0;
                    y0 = y1;
                    y1 = tp;
                    swap_x = true;
                }

                dx = x1 - x0;
                dy = y1 - y0;
                d_err = 2 * abs(dy);
                err = 0;
                y = y0;
                y_step = y0 < y1? 1:-1;

                //3.4.1 detect obstacle
                no_obs = true;
                for(I4 xi = x0;xi <= x1;xi++){
                    if(!param.obstacle_avoid){
                        if(kover1){
                            tmp_i = xi*width + y;
                        }else{
                            tmp_i = y*width + xi;
                        }
                        if(map.data[tmp_i] < 0 || map.data[tmp_i] > param.obstacle_prob){
                            no_obs = false;
                        }
                    } else {
                        for (I4 xa = xi - 1; xa <= xi + 1; xa++) {
                            for (I4 ya = y - 1; ya <= y + 1; ya++) {
                                if (kover1) {
                                    if(ya >= width || ya < 0 || xa >= height || xa < 0){
                                        continue;
                                    }
                                    tmp_i = xa * width + ya;
                                } else {
                                    if(xa >= width || xa < 0 || ya >= height || ya < 0){
                                        continue;
                                    }
                                    tmp_i = ya * width + xa;
                                }
                                if (map.data[tmp_i] < 0 || map.data[tmp_i] > param.obstacle_prob) {
                                    no_obs = false;
                                }
                            }
                        }
                    }
                    if(!no_obs){
                        break;
                    }
                    err += d_err;
                    if(err > dx){
                        y += y_step;
                        err -= (2*dx);
                    }
                }

                //3.4.2 if no obs ,update three, find min dis node
                if(no_obs){
                    //3.4.2 DFS judge dis
                    has_one = true;
                    index_list.clear();
                    DFS(node_i,init_j,pre);
                    R8 now_dis = pointSquaredDistance[now_k]*pointSquaredDistance[now_k];
                    I4 xf,yf,xb,yb;
                    for(I4 v = 1;v<index_list.size();v++){
                        xf = index_list[v]%width;
                        yf = index_list[v]/width;
                        xb = index_list[v-1]%width;
                        yb = index_list[v-1]/width;
                        now_dis += ((xf-xb)*(xf-xb) + (yf-yb)*(yf-yb));
                    }
                    if(now_dis < min_dis){
                        min_dis = now_dis;
                        min_i = node_i;
                    }
                }
                now_k++;
            }

            //3,5 if has one update
            if(has_one){
                pre[search_i] = min_i;
                end_i = search_i;
                searchPoint.x = (R8)(search_i%width);
                searchPoint.y = (R8)(search_i/width);
                cloud->points.push_back(searchPoint);
                kdtree.setInputCloud(cloud);
                dis = (searchPoint.x - final_gx)*(searchPoint.x - final_gx) +
                      (searchPoint.y - final_gy)*(searchPoint.y - final_gy) ;
            }
            try_times++;
            pointIdSearch.clear();
            pointSquaredDistance.clear();
        }

        //4.get path
        std::vector<Pose_6d> path_get;
        if(try_times >= param.max_try_times){
            printf("cannot find path");
            path_get.push_back(init_pose);
            return path_get;
        }
        index_list.clear();
        DFS(end_i,init_j,pre);
        Pose_6d tmp_pose;
        U4 path_size = index_list.size();
        tmp_pose.x = init_pose.x;
        tmp_pose.y = init_pose.y;
        path_get.push_back(tmp_pose);
        for(I4 i = path_size-1;i >= 0;i--){
            tmp_pose.x = (index_list[i]%width)*rslo + ox + rslo/2;
            tmp_pose.y = (index_list[i]/width)*rslo + oy + rslo/2;
            path_get.push_back(tmp_pose);
        }
        tmp_pose.x = final_pose.x;
        tmp_pose.y = final_pose.y;
        path_get.push_back(tmp_pose);
        return path_get;
    }

    std::vector<Pose_6d> Planner::Sweep_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                           Pose_6d &final_pose,Plan_Param &param){
        R8 ox     = map.info.origin.position.x;
        R8 oy     = map.info.origin.position.y;
        R8 rslo   = map.info.resolution;
        U4 width  = map.info.width;
        U4 height = map.info.height;

        I4 init_gx  = floor((init_pose.x - ox)/rslo);
        I4 init_gy  = floor((init_pose.y - oy)/rslo);
        I4 final_gx = floor((final_pose.x - ox)/rslo);
        I4 final_gy = floor((final_pose.y - oy)/rslo);
        I4 init_j   = init_gy*width + init_gx;
        I4 final_j  = final_gy*width + final_gx;
        if(map.data[final_j]<0 || map.data[final_j]>param.obstacle_prob){
            assert(false);
        }
        if(final_gx<0 || final_gx>=width || final_gy<0 || final_gy>=height){
            assert(false);
        }

        //1.init
        index_list.clear();
        Pose_6d tmp_pose;
        std::vector<Pose_6d> path_get;
        std::vector<Pose_6d> tmp_path;
        U4 map_size = map.data.size();
        B1 occupied[map_size];
        I4 nx = init_gx;
        I4 ny = init_gy;
        for(I4 i = 0;i<map_size;i++){
            occupied[i] = !(map.data[i] >= 0 && map.data[i] <= param.obstacle_prob);
        }
        B1 block,over;

        //1.1 kdtree init
        pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        std::vector<int> pointIdSearch;
        std::vector<float> pointSquaredDistance;
        pcl::PointXY tmp{0,0};
        pcl::PointXY searchPoint{0,0};
        Pose_6d I_pose,F_pose;
        R4 radius = ceil(param.radius/rslo);
        over = false;
        while(true){
            block = true;
            //2.right first
            nx++;
            while(nx>=0 && nx < width && map.data[ny*width + nx] >= 0 && map.data[ny*width + nx] <= param.obstacle_prob){
                if(nx>=0 && nx < width && !occupied[ny*width + nx]){
                    tmp_pose.x = nx*rslo + ox + rslo/2;
                    tmp_pose.y = ny*rslo + oy + rslo/2;
                    path_get.push_back(tmp_pose);
                    occupied[ny*width + nx] = true;
                    nx ++;
                    block = false;
                } else {
                    break;
                }
            }
            nx--;

            //3.left then
            nx--;
            while(nx>=0 && nx < width && map.data[ny*width + nx] >= 0 && map.data[ny*width + nx] <= param.obstacle_prob){
                if(!occupied[ny*width + nx]){
                    tmp_pose.x = nx*rslo + ox + rslo/2;
                    tmp_pose.y = ny*rslo + oy + rslo/2;
                    path_get.push_back(tmp_pose);
                    occupied[ny*width + nx] = true;
                    nx --;
                    block = false;
                } else {
                    break;
                }
            }
            nx++;

            //4.up then
            ny++;
            if(ny>=0 && ny < height && map.data[ny*width + nx] >= 0 && map.data[ny*width + nx] <= param.obstacle_prob){
                if(!occupied[ny*width + nx]){
                    tmp_pose.x = nx*rslo + ox + rslo/2;
                    tmp_pose.y = ny*rslo + oy + rslo/2;
                    path_get.push_back(tmp_pose);
                    occupied[ny*width + nx] = true;
                    ny ++;
                    block = false;
                }
            }
            ny--;

            //5.down then
            ny--;
            if(ny>=0 && ny < height && map.data[ny*width + nx] >= 0 && map.data[ny*width + nx] <= param.obstacle_prob){
                if(!occupied[ny*width + nx]){
                    tmp_pose.x = nx*rslo + ox + rslo/2;
                    tmp_pose.y = ny*rslo + oy + rslo/2;
                    path_get.push_back(tmp_pose);
                    occupied[ny*width + nx] = true;
                    ny --;
                    block = false;
                }
            }
            ny++;
            //6.if block,use A* or Dijkstra
            if(block){
                //6.1 find nearest available
                U4 i = 0;
                cloud->points.clear();
                over = true;
                while(i<map_size){
                    if(!occupied[i]){
                        tmp.x = (R8)(i%width);
                        tmp.y = (R8)(i/width);
                        cloud->points.push_back(tmp);
                        over = false;
                    }
                    i++;
                }
                if(!over){
                    kdtree.setInputCloud(cloud);
                    searchPoint.x = (R8)nx;
                    searchPoint.y = (R8)ny;
                    pointIdSearch.clear();
                    pointSquaredDistance.clear();
                    if(param.KNsearch){
                        if(kdtree.nearestKSearch (searchPoint, param.K, pointIdSearch, pointSquaredDistance)>0){}
                    }
                    if(!param.KNsearch){
                        if(kdtree.radiusSearch (searchPoint, radius, pointIdSearch, pointSquaredDistance)>0){}
                    }
                    F_pose.x = cloud->points[ pointIdSearch[0] ].x*rslo + ox + rslo/2;
                    F_pose.y = cloud->points[ pointIdSearch[0] ].y*rslo + ox + rslo/2;
                }
                //6.2 find path to get it;
                I_pose.x = nx*rslo + ox + rslo/2;
                I_pose.y = ny*rslo + oy + rslo/2;
                if(over){
                    F_pose.x = final_gx*rslo + ox + rslo/2;
                    F_pose.y = final_gy*rslo + oy + rslo/2;
                }
                tmp_path.clear();
                switch (param.block_Search) {
                    case Dijkstra:
                        tmp_path = Dijkstra_2D(map,I_pose,F_pose,param);
                        break;
                    case A_Star:
                        tmp_path = A_Star_2D(map,I_pose,F_pose,param);
                        break;
                    case RPM:
                        tmp_path = RPM_2D(map,I_pose,F_pose,param);
                        break;
                    case RRT:
                        tmp_path = RRT_2D(map,I_pose,F_pose,param);
                        break;
                    case RRT_Star:
                        tmp_path = RRT_Star_2D(map,I_pose,F_pose,param);
                        break;
                    default:
                        printf("Plan Mode Error,Please check plan mode!");
                        assert(false);
                        break;
                }
                //6.2 update path
                path_get.insert(path_get.end(),tmp_path.begin(),tmp_path.end());
                if(over){break;}
                nx = cloud->points[ pointIdSearch[0] ].x;
                ny = cloud->points[ pointIdSearch[0] ].y;
                occupied[ny*width + nx] = true;
            }
        }

        return path_get;
    }

    std::vector<Pose_6d> Planner::Map_Refer_Simulation(Pose_6d &init_pose,Plan_Param &param){

        std::vector<Pose_6d> path_get;
        R8 v = param.sim_v;
        R8 dt = param.sim_dt;
        R8 steer = 0;
        Pose_6d tmp_pose = init_pose;
        tmp_pose.v = v;
        for(R8 t = 0;t<100;t+=dt){
            tmp_pose.x += v*cos(tmp_pose.yaw)*dt;
            tmp_pose.y += v*sin(tmp_pose.yaw)*dt;
            tmp_pose.yaw += v/param.sim_wb*tan(steer)*dt;
            path_get.push_back(tmp_pose);
            if(t >= 50 && t<=65){
                steer = 0.43;
            }
            if(t>65){
                steer = 0;
            }
        }
        return path_get;
    }

    std::vector<Pose_6d> Planner::Dwa_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                         Plan_Param &param,std::vector<Pose_6d> &refer_line){
        //1.init
        R8 ox     = map.info.origin.position.x;
        R8 oy     = map.info.origin.position.y;
        R8 rslo   = map.info.resolution;
        U4 width  = map.info.width;
        U4 height = map.info.height;

        I4 init_gx  = floor((init_pose.x - ox)/rslo);
        I4 init_gy  = floor((init_pose.y - oy)/rslo);
        I4 init_j   = init_gy*width + init_gx;

        R8 w_scale = param.w_scale;
        R8 w_intv = param.w_interval;
        R8 v_scale = param.v_scale;
        R8 v_intv = param.v_interval;
        R8 dt = param.dt;
        U4 map_size = map.data.size();

        //2.construct kdtree for obstacle
        pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
        pcl::KdTreeFLANN<pcl::PointXY> kdtree;
        pcl::PointXY tmp{0,0};
        pcl::PointXY searchPoint{0,0};
        cloud->height = 1;
        cloud->width  = 1;
        for(U4 i = 0;i < map_size;i++){
            if(map.data[i]<0 || map.data[i]>param.obstacle_prob){
                tmp.x = (R8)(i%width);
                tmp.y = (R8)(i/width);
                cloud->points.push_back(tmp);
            }
        }
        kdtree.setInputCloud(cloud);
        std::vector<int> pointIdSearch;
        std::vector<float> pointSquaredDistance;
        R4 radius = ceil(param.radius/rslo);

        //3.sample point
        R8 now_x = init_pose.x;
        R8 now_y = init_pose.y;
        R8 theta = init_pose.yaw;
        R8 x_end=0,y_end=0,th_end=0;
        R8 vbw,wdt;
        R8 t_intv = fabs(rslo/init_pose.v/4);
        R8 block_dis = param.min_block_dis/rslo;
        R8 t;
        R8 min_dis;
        R8 mm_dis;
        B1 over;
        B1 this_block;
        R8 id_min_dis;
        R8 id_dis;
        R8 min_path_loss = 99999999;
        R8 path_loss;
        R8 ap = param.ap;
        R8 bp = param.bp;
        R8 cp = param.cp;
        R8 min_v=0.0001,min_w=0.0001;
        I4 min_i=0;
        B1 k_find;
        if(t_intv>=dt/2){t_intv = dt/2;}
        for(R8 w=init_pose.w-w_scale;w<=init_pose.w+w_scale;w+=w_intv){
            for(R8 v=init_pose.v-v_scale;v<=init_pose.v+v_scale;v+=v_intv){
                t = 0;
                over = false;
                this_block = false;
                //3.for count min dis
                mm_dis = 99999999;
                while(t < dt && !over){
                    t += t_intv;
                    if(t>=dt){
                        t=dt;
                        over = true;
                    }
                    vbw = v/w;
                    wdt = w*t;
                    x_end = now_x - vbw*sin(theta) + vbw*sin(theta+wdt);
                    y_end = now_y + vbw*cos(theta) - vbw*cos(theta+wdt);
                    searchPoint.x = (x_end-ox)/rslo; //in grid axis
                    searchPoint.y = (y_end-oy)/rslo;
                    k_find = false;
                    if(param.KNsearch){
                        if(kdtree.nearestKSearch (searchPoint, param.K, pointIdSearch, pointSquaredDistance)>0){k_find = true;}
                    }
                    if(!param.KNsearch){
                        if(kdtree.radiusSearch (searchPoint, radius, pointIdSearch, pointSquaredDistance)>0){k_find = true;}
                    }
                    if(!k_find){
                        min_dis = 9999999;
                    } else {
                        min_dis = pointSquaredDistance[0];
                    }
                    if(min_dis<block_dis){
                        this_block = true;
                        break;
                    }
                    if(min_dis < mm_dis){
                        mm_dis = min_dis;
                    }
                }
                if(this_block){
                    continue;
                }
                th_end = theta + w*dt;
                //4.find end index
                id_min_dis = 999999999;
                for(I4 i = param.init_index;i<=param.end_index;i++){
                    id_dis = (refer_line[i].x - x_end)*(refer_line[i].x - x_end) +
                             (refer_line[i].y - y_end)*(refer_line[i].y - y_end);
                    if(id_dis < id_min_dis){
                        id_min_dis = id_dis;
                        min_i = i;
                    }
                }
                //5.count score
                R8 dyaw = yaw_sub(refer_line[min_i].yaw,th_end);
                path_loss = ap/mm_dis +
                            bp*dyaw*dyaw +
                            cp*(refer_line[min_i].y - y_end)*(refer_line[min_i].y - y_end) +
                            cp*(refer_line[min_i].x - x_end)*(refer_line[min_i].x - x_end) ;
                if(path_loss < min_path_loss){
                    min_path_loss = path_loss;
                    min_v = v;
                    min_w = w;
                }
            }
        }
        std::vector<Pose_6d> path_get;
        Pose_6d path_tmp;
        t = 0;
        if(min_v == 0.0001 && min_w == 0.0001){
            printf("Get local path failed\n");
            path_get.push_back(init_pose);
            return path_get;
        }
        while(t < dt){
            t+= param.out_interval;
            vbw = min_v/min_w;
            wdt = min_w*t;
            path_tmp.x = now_x - vbw*sin(theta) + vbw*sin(theta+wdt);
            path_tmp.y = now_y + vbw*cos(theta) - vbw*cos(theta+wdt);
            path_tmp.yaw = wdt;
            path_tmp.v = min_v;
            path_tmp.w = min_w;
            path_get.push_back(path_tmp);
        }
        return path_get;
    }

    std::vector<Pose_6d> Planner::State_Lattice_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                                   Plan_Param &param,std::vector<Pose_6d> &refer_line){
        //1.find nearest point
        U4 line_size = refer_line.size();
        R8 min_dis = 9999999999;
        R8 tmp_dis;
        U4 min_i=0;
        for(U4 i = param.init_index;i<=param.end_index;i++){
            tmp_dis = (init_pose.x - refer_line[i].x)*(init_pose.x - refer_line[i].x) -
                      (init_pose.y - refer_line[i].y)*(init_pose.y - refer_line[i].y) ;
            if(tmp_dis < min_dis){
                min_dis = tmp_dis;
                min_i = i;
            }
        }

        //2.get vertical distance，and frenet orgin
        std::vector<Pose_6d> path_get;
        Pose_6d frenet_orgin;
        Pose_6d frenet_now_pose;
        R8 v_dis; //vertical dis
        R8 ds;
        R8 l = param.path_v*param.path_dt;
        U4 sub_i,low_i;
        if(min_i == 0 || min_i == line_size-1){
            path_get.push_back(init_pose);
            return path_get;
        } else {
            R8 up_dis,down_dis;
            R8 dl,ul;
            R8 cosA;
            R8 sinA;
            up_dis = (init_pose.x - refer_line[min_i+1].x)*(init_pose.x - refer_line[min_i+1].x) -
                     (init_pose.y - refer_line[min_i+1].y)*(init_pose.y - refer_line[min_i+1].y) ;
            down_dis = (init_pose.x - refer_line[min_i-1].x)*(init_pose.x - refer_line[min_i-1].x) -
                       (init_pose.y - refer_line[min_i-1].y)*(init_pose.y - refer_line[min_i-1].y) ;
            sub_i = up_dis<down_dis?(min_i+1):(min_i-1);
            low_i = sub_i<min_i?sub_i:min_i;
            ul = up_dis<down_dis?up_dis:min_dis;
            dl = up_dis<down_dis?min_dis:down_dis;
            cosA = (l*l + dl*dl - ul*ul)/(2*l*dl);
            sinA = sqrt(1-cosA*cosA);
            v_dis = dl*sinA;
            ds = dl*cosA;
            frenet_orgin.yaw = refer_line[low_i].yaw;
            frenet_orgin.x = refer_line[low_i].x + ds*cos(frenet_orgin.yaw);
            frenet_orgin.y = refer_line[low_i].y + ds*sin(frenet_orgin.yaw);

            R8 to_yaw = get_angle(refer_line[low_i].y,refer_line[low_i].x,init_pose.y,init_pose.x);
            to_yaw = yaw_sub(to_yaw,refer_line[low_i].yaw);
            if( (to_yaw>=0 && to_yaw<TT) ){
                v_dis = -v_dis;
            }

            frenet_now_pose.y = v_dis;
            frenet_now_pose.x = 0;
            frenet_now_pose.yaw = yaw_sub(init_pose.yaw,refer_line[low_i].yaw);
            frenet_now_pose.v = init_pose.v;
            frenet_now_pose.a = init_pose.a;
        }

        //3.sample lateral
        std::vector<R8> a_latl,b_latl,c_latl,d_latl,e_latl,f_latl;
        R8 v_lat = frenet_now_pose.v*sin(frenet_now_pose.yaw);
        R8 v_lon = frenet_now_pose.v*cos(frenet_now_pose.yaw);
        R8 a_lat = 0;
        R8 a_lon = frenet_now_pose.a;
        R8 a,b,c,d,e,f;
        Eigen::MatrixXd A(3,3);
        Eigen::Vector3d B;
        Eigen::Vector3d X;
        R8 s12,s13,s14,s15;
        for(R8 l1 = param.l1_scale;l1 <= param.l1_scalep;l1+=param.l1_intv){
            for(R8 s1 = param.s1_scale;s1 <= param.s1_scalep;s1+=param.s1_intv){
                f = frenet_now_pose.y;
                e = v_lat/v_lon;
                d = 0;
                B << (l1-e*s1-f),
                     (   -e    ),
                     (    0    );
                s12 = s1*s1;
                s13 = s12*s1;
                s14 = s13*s1;
                s15 = s14*s1;
                A << (s15)   ,(s14)   ,(s13)  ,
                     (5*s14) ,(4*s13) ,(3*s12),
                     (20*s13),(12*s12),(6*s1) ;
                if(A.determinant() == 0){
                    continue;
                }
                X = A.inverse()*B;
                a = X(0);
                b = X(1);
                c = X(2);
                f_latl.push_back(f);
                e_latl.push_back(e);
                d_latl.push_back(d);
                c_latl.push_back(c);
                b_latl.push_back(b);
                a_latl.push_back(a);
            }
        }

        //4.sample longitude
        Eigen::MatrixXd A2(2,2);
        Eigen::Vector2d B2;
        Eigen::Vector2d X2;
        std::vector<R8> a_lonl,b_lonl,c_lonl,d_lonl,e_lonl;
        R8 t13,t12;
        for(R8 s1d = param.s1d_scale;s1d <= param.s1d_scalep;s1d+=param.s1d_intv){
            for(R8 t1 = param.t1_scale;t1 <= param.t1_scalep;t1+=param.t1_intv){
                e = 0;
                d = v_lon;
                c = a_lon;
                t12 = t1*t1;
                t13 = t12*t1;
                B2 << (s1d-d-2*c*t1),
                      (    -2*c    );
                A2 << (4*t13) ,(3*t12),
                      (12*t12),(6*t1) ;
                if(A2.determinant() == 0){
                    continue;
                }
                X2 = A2.inverse()*B2;
                a = X2(0);
                b = X2(1);

                e_lonl.push_back(e);
                d_lonl.push_back(d);
                c_lonl.push_back(c);
                b_lonl.push_back(b);
                a_lonl.push_back(a);
            }
        }

        //5.mix and evalue
        Pose_6d tmp_fre_pose;
        R8 t14;
        R8 s,s1d,s2d;
        R8 la,la1d,la2d;
        R8 s2,s3,s4,s5;
        B1 obs;
        R8 min_path_loss = 9999999999999;
        R8 path_loss;
        R8 af = param.af;
        R8 bf = param.bf;
        R8 cf = param.cf;
        R8 i_min=0,j_min=0;
        for(U4 i = 0;i<a_latl.size();i++){
            for(U4 j = 0;j<a_lonl.size();j++){
                obs = true;
                for(R8 t=0;t<=param.t1_scalep;t+=param.sample_intv){
                    t12 = t*t;
                    t13 = t12*t;
                    t14 = t13*t;
                    s = a_lonl[j]*t14 + b_lonl[j]*t13 + c_lonl[j]*t12 +
                        d_lonl[j]*t + e_lonl[j];
                    s1d = 4*a_lonl[j]*t13 + 3*b_lonl[j]*t12 + 2*c_lonl[j]*t +
                          d_lonl[j];
                    s2d = 12*a_lonl[j]*t12 + 6*b_lonl[j]*t + 2*c_lonl[j];

                    s2 = s*s;
                    s3 = s2*s;
                    s4 = s3*s;
                    s5 = s4*s;
                    la = a_latl[i]*s5 + b_latl[i]*s4 + c_latl[i]*s3 + d_latl[i]*s2 +
                         e_latl[i]*s + f_latl[i];
                    la1d = 5*a_latl[i]*s4 + 4*b_latl[i]*s3 + 3*c_latl[i]*s2 + 2*d_latl[i]*s +
                           e_latl[i];
                    la2d = 20*a_latl[i]*s3 + 12*b_latl[i]*s2 + 6*c_latl[i]*s + 2*d_latl[i];
                    tmp_fre_pose.x = s;
                    tmp_fre_pose.y = la;
                    tmp_fre_pose.v = sqrt(s1d*s1d + la1d*la1d);
                    tmp_fre_pose.a = s2d;
                    tmp_fre_pose.yaw = tan(la1d/s1d);
                    //5.1 count min obstacle distance
                    //as we dont have perception layer,this loss ignore
                    //for achieve this，refer to dwa,
                    //now we assume no obstacle
                    obs = false;
                }
                if(!obs){
                    path_loss = af*tmp_fre_pose.yaw*tmp_fre_pose.yaw +
                                bf*tmp_fre_pose.y*tmp_fre_pose.y +
                                cf/(tmp_fre_pose.x*tmp_fre_pose.x);
                    if(path_loss < min_path_loss){
                        min_path_loss = path_loss;
                        i_min = i;
                        j_min = j;
                    }
                }
            }
        }

        //6.select best and trans from frenet to world axis
        Pose_6d tmp_pose;
        U4 i = i_min;
        U4 j = j_min;
        R8 dis_total = 0;
        U4 now_i = low_i;
        R8 cx,cy; //center x ,center y
        R8 dx,dy;
        R8 ldx,ldy;
        R8 dh;
        R8 dr_yaw=0;
        for(R8 t=param.out_t_intv;t<=param.t1_scalep;t+=param.out_t_intv){
            t12 = t*t;
            t13 = t12*t;
            t14 = t13*t;
            s = a_lonl[j]*t14 + b_lonl[j]*t13 + c_lonl[j]*t12 +
                d_lonl[j]*t + e_lonl[j];
            s1d = 4*a_lonl[j]*t13 + 3*b_lonl[j]*t12 + 2*c_lonl[j]*t +
                  d_lonl[j];
            s2d = 12*a_lonl[j]*t12 + 6*b_lonl[j]*t + 2*c_lonl[j];

            s2 = s*s;
            s3 = s2*s;
            s4 = s3*s;
            s5 = s4*s;
            la = a_latl[i]*s5 + b_latl[i]*s4 + c_latl[i]*s3 + d_latl[i]*s2 +
                 e_latl[i]*s + f_latl[i];
            la1d = 5*a_latl[i]*s4 + 4*b_latl[i]*s3 + 3*c_latl[i]*s2 + 2*d_latl[i]*s +
                   e_latl[i];
            la2d = 20*a_latl[i]*s3 + 12*b_latl[i]*s2 + 6*c_latl[i]*s + 2*d_latl[i];

            //7. trans frenet to world axis
            //7.1 get now_i
            R8 s_a = s + ds;
            while(dis_total < s_a){
                dis_total += l;
                now_i++;
            }
            R8 res = l - (dis_total - s_a);
            dis_total -= l;
            now_i--;

            //7.2 count center and p
            dx = res*cos(refer_line[now_i].yaw);
            dy = res*sin(refer_line[now_i].yaw);
            cx = refer_line[now_i].x + dx;
            cy = refer_line[now_i].y + dy;
            dx = fabs(dx);
            dy = fabs(dy);
            dh = fabs(la);
            ldx = dh*dy/res;
            ldy = dh*dx/res;
            if(la > 0){
                dr_yaw = yaw_add(refer_line[now_i].yaw,TT/2);
            } else if (la == 0){
                dr_yaw = refer_line[now_i].yaw;
            } else if (la < 0){
                dr_yaw = yaw_sub(refer_line[now_i].yaw,TT/2);
            }
            if(dr_yaw >=TT/2 && dr_yaw < TT){
                ldx = -ldx;
            }
            if(dr_yaw >=TT && dr_yaw < 3*TT/2){
                ldx = -ldx;
                ldy = -ldy;
            }
            if(dr_yaw >=3*TT/2 && dr_yaw < 2*TT){
                ldy = -ldy;
            }
            tmp_pose.x = cx + ldx;
            tmp_pose.y = cy + ldy;
            tmp_pose.v = sqrt(s1d*s1d + la1d*la1d);
            tmp_pose.a = s2d;
            tmp_pose.yaw = yaw_add(tan(la1d/s1d),refer_line[now_i].yaw);
            path_get.push_back(tmp_pose);
        }

        //7.output
        return path_get;
    }


    std::vector<Pose_6d> Planner::Reeds_Shepp_2D(nav_msgs::OccupancyGrid &map,Pose_6d &init_pose,
                                                 Plan_Param &param,std::vector<Pose_6d> &refer_line){
        //0.interpolation
        std::vector<Pose_6d> path_get;
        std::vector<Pose_6d> refer_now;
        if(param.pre_intp){
            refer_now = Linear_Interpolation(refer_line,param.intp_scale);
        } else{
            refer_now = refer_line;
        }
        //1.find nearest point
        U4 line_size = refer_now.size();
        R8 min_dis = 9999999999;
        R8 tmp_dis;
        U4 min_i=0;
        for(U4 i = param.init_index;i<=param.end_index;i++){
            tmp_dis = (init_pose.x - refer_now[i].x)*(init_pose.x - refer_now[i].x) -
                      (init_pose.y - refer_now[i].y)*(init_pose.y - refer_now[i].y) ;
            if(tmp_dis < min_dis){
                min_dis = tmp_dis;
                min_i = i;
            }
        }
        init_pose.yaw = refer_now[min_i].yaw;

        //2.search candidate point
        R8 stop_yaw = param.stop_pose.yaw;
        R8 wheel_base = param.wheel_base;
        R8 steer = param.max_steer;
        Pose_6d now_pose;
        Pose_6d stop_pose = param.stop_pose;
        //2.1 judege steer direction
        R8 to_yaw = get_angle(stop_pose.y,stop_pose.x,init_pose.y,init_pose.x);
        to_yaw = yaw_sub(to_yaw,init_pose.yaw);
        if( !(to_yaw>=0 && to_yaw<TT) ){
            steer = -steer;
        }

        R8 back_v = param.idling;
        now_pose.v = back_v;
        R8 t;
        R8 dyaw = 0;
        B1 over = false;
        I4 find_i = -1;
        R8 min_x_bias = 9999999;
        R8 end_theta;
        R8 x_bias,y_bias;
        R8 sx_bias = param.sx_bias*param.sx_bias;
        R8 sy_bias = param.sy_bias*param.sy_bias;
        R8 s=0;
        if(min_i+param.max_search >= line_size){
            printf("max_over!\n");
            path_get.push_back(init_pose);
            return path_get;
        }
        for(U4 i = min_i;i<min_i+param.max_search;i++){
            now_pose = refer_now[i];
            t = 0;
            //2.2 detect path
            while(dyaw < TT){
                now_pose.x += back_v*cos(now_pose.yaw)*t;
                now_pose.y += back_v*sin(now_pose.yaw)*t;
                dyaw = back_v/wheel_base*tan(steer)*t;
                now_pose.yaw += dyaw;
                dyaw = fabs(dyaw);
                t += param.t_interval;
                if(fabs(yaw_sub(now_pose.yaw,stop_yaw))<=param.yaw_bias){
                    over = true;
                    break;
                }
            }
            //2.3 count x bias
            if(over){
                if(now_pose.x == stop_pose.x){
                    end_theta = now_pose.y>stop_pose.y?1.57:-1.57;
                } else {
                    end_theta = get_angle(now_pose.y,now_pose.x,stop_pose.y,stop_pose.x);
                }
                end_theta = end_theta - stop_pose.yaw;
                s = (now_pose.y-stop_pose.y)*(now_pose.y-stop_pose.y) +
                         (now_pose.x-stop_pose.x)*(now_pose.x-stop_pose.x) ;
                y_bias = s*cos(end_theta);
                x_bias = fabs(sin(end_theta)*s);
                if(x_bias >= sx_bias || y_bias < sy_bias){
                    continue;
                }
                if(x_bias < min_x_bias){
                    min_x_bias = x_bias;
                    find_i = i;
                }
            }
        }

        //3.output
        if(!over || find_i == -1){
            printf("Get local path failed,over:%d,find:%d\n",over,find_i);
            path_get.push_back(init_pose);
            return path_get;
        }

        for(U4 i = min_i;i<=find_i;i++){
            path_get.push_back(refer_now[i]);
        }
        t = 0;
        now_pose = refer_now[find_i];
        R8 old_yaw_d,new_yaw_d;
        U4 try_t=0;
        //3.1 curve
        while(try_t <= 100000){
            old_yaw_d = fabs(yaw_sub(now_pose.yaw,stop_yaw));
            now_pose.x += back_v*cos(now_pose.yaw)*t;
            now_pose.y += back_v*sin(now_pose.yaw)*t;
            now_pose.yaw += back_v/wheel_base*tan(steer)*t;
            new_yaw_d = fabs(yaw_sub(now_pose.yaw,stop_yaw));
            if(old_yaw_d < new_yaw_d){
                break;
            }
            path_get.push_back(now_pose);
            try_t++;
            t += param.out_t;
        }

        //3.2 straight line
        std::vector<Pose_6d> path_straight;
        R8 res_min = param.res_bias*param.res_bias;
        R8 old_res;
        R8 sv = -back_v;
        now_pose.yaw = stop_yaw;
        t = 0;
        R8 res_dis=9999999;
        while(res_dis >= res_min){
            stop_pose.x += sv*cos(now_pose.yaw)*t;
            stop_pose.y += sv*sin(now_pose.yaw)*t;
            t += param.out_t;
            old_res = res_dis;
            res_dis = (now_pose.y-stop_pose.y)*(now_pose.y-stop_pose.y) +
                      (now_pose.x-stop_pose.x)*(now_pose.x-stop_pose.x) ;
            if(res_dis >= old_res){
                break;
            }
            path_straight.push_back(stop_pose);
        }
        for(I4 i = path_straight.size()-1;i>=1;i--){
            path_get.push_back(path_straight[i]);
        }
        return  path_get;
    }
}
