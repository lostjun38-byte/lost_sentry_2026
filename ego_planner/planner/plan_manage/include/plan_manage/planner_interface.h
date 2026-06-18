/*
 * @Function:Ego Planner:Trajectory Optimize Base Bspline
 * @Create by:juchunyu@qq.com
 * @Date:2025-08-02 17:40:01
 */

#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <stdlib.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <chrono>


namespace  ego_planner
{
    struct PathPoint
    {
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
        float v{0.0F};
    };

    struct ObstacleInfo
    {
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
    };

    class PlannerInterface
    {
           
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                                Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                                Eigen::Vector3d local_target_vel, Eigen::Vector3d local_target_acc,
                                                vector<Eigen::Vector3d> point_set);
            PlanParameters pp_;
            LocalTrajData local_data_;
            shared_ptr<GridMap2D> grid_map_;
            PathPoint  cur_pose_;
            bool map_is_preinflated_{false};

        private:

            BsplineOptimizer::Ptr bspline_optimizer_rebound_;

            int continous_failures_count_{0};

            // Current allocated A* node pool size. The pool must be large
            // enough to cover the live OccupancyGrid plus a margin; otherwise
            // AstarSearch fails the moment a costmap resize crosses the bound.
            Eigen::Vector2i a_star_pool_size_{0, 0};

            // Rebuild bspline_optimizer_rebound_->a_star_ if pool is smaller
            // than required. Safe to call from any context that already holds
            // the planner_mutex_ in the caller (this class itself does not
            // own the mutex).
            void ensureAStarPoolCovers(const Eigen::Vector2i & required);

            void updateTrajInfo(const UniformBspline &position_traj);

            void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                            double &time_inc);

            bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);
   
            
        private:

           
            std::vector<PathPoint> _global_plan_traj_;
            std::vector<PathPoint> _plan_traj_results_;
            vector<vector<Eigen::Vector2d>> a_star_pathes_;



        public:
            PlannerInterface();

            ~PlannerInterface();

            void initParam(double max_vel,double max_acc,double max_jerk);

            void initEsdfMap(double x_size,double y_size,double z_size,double resolution, Eigen::Vector3d org,double inflate_values);
            void setOccupancyGridMap(
                const nav_msgs::msg::OccupancyGrid& costmap,
                double inflate_values,
                int occupied_cost_threshold = 50);
            void setNav2InflatedOccupancyGridMap(
                const nav_msgs::msg::OccupancyGrid& costmap,
                int occupied_cost_threshold = 50);

            void setPathPoint(std::vector<PathPoint> &plan_traj);

            void setObstacles(std::vector<ObstacleInfo> &obstacle);

            void setCurrentVehiclePos(PathPoint& cur_pose);

            void makePlan();

            void getLocalPlanTrajResults(std::vector<PathPoint> &plan_traj_results);  

            void getObstacles(std::vector<ObstacleInfo> &obstacle);

            void getTraj();
            void clearPlanTrajResults();
            bool isWorldPointOccupied(double x, double y) const;

            void setGridMap(PathPoint& cur_pose);

            void getAStarPath(vector<vector<Eigen::Vector2d>>& a_star_path);

    };


}


#endif
