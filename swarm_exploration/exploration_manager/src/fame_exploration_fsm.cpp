
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fame_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fame_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/HGrid.h>
#include <exploration_manager/GridTour.h>

#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/ReferenceStampedSrv.h>

#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>



#include <std_msgs/Bool.h>
#include <fstream>

using Eigen::Vector4d;

namespace fast_planner {

void FameExplorationFSM::init(ros::NodeHandle& nh) {
  fame_params_.reset(new FSMParam);
  fame_data_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fame_params_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fame_params_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fame_params_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fame_params_->replan_time_, -1.0);
  nh.param("fsm/attempt_interval", fame_params_->attempt_interval_, 0.2);
  nh.param("fsm/pair_opt_interval", fame_params_->pair_opt_interval_, 1.0);
  nh.param("fsm/repeat_send_num", fame_params_->repeat_send_num_, 10);
  nh.param(
      "fsm/communication_range", fame_params_->communication_range_, std::numeric_limits<double>::max());

  /* Initialize main modules */
  expl_manager_.reset(new FameExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));
  coll_assigner_.reset(new CollaborationAssigner(nh));




  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;

  fame_data_->have_odom_ = false;
  fame_data_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH","IDLE" };
  fame_data_->static_state_ = true;
  fame_data_->trigger_ = false;
  fame_data_->avoid_collision_ = true;
  fame_data_->go_back_ = false;

  num_fail_ = 0;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FameExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FameExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.1), &FameExplorationFSM::frontierCallback, this);
  heartbit_timer_ = nh.createTimer(ros::Duration(1.0), &FameExplorationFSM::heartbitCallback, this);

  odom_sub_ = nh.subscribe("/odom_world", 1, &FameExplorationFSM::odometryCallback, this);
  idle_sub = nh.subscribe("/diagnostics", 1, &FameExplorationFSM::diagnosticCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  stop_pub_ = nh.advertise<std_msgs::Int32>("/stop", 1000);
  heartbit_pub_ = nh.advertise<std_msgs::Empty>("/heartbit", 100);
  pose_pub_ = nh.advertise<geometry_msgs::Pose>("/fame/explore_target", 10);


  emergency_handler_pub_ = nh.advertise<std_msgs::Bool>("/trigger_emergency", 10);

  // Swarm, timer, pub and sub
  drone_state_timer_ =
      nh.createTimer(ros::Duration(0.04), &FameExplorationFSM::droneStateTimerCallback, this);
  drone_state_pub_ =
      nh.advertise<exploration_manager::DroneState>("/swarm_expl/drone_state_send", 10);
  drone_state_sub_ = nh.subscribe(
      "/swarm_expl/drone_state_recv", 10, &FameExplorationFSM::droneStateMsgCallback, this);

  opt_timer_ = nh.createTimer(ros::Duration(0.05), &FameExplorationFSM::optTimerCallback, this);
  opt_pub_ = nh.advertise<exploration_manager::PairOpt>("/swarm_expl/pair_opt_send", 10);
  opt_sub_ = nh.subscribe("/swarm_expl/pair_opt_recv", 100, &FameExplorationFSM::optMsgCallback,
      this, ros::TransportHints().tcpNoDelay());

  opt_res_pub_ =
      nh.advertise<exploration_manager::PairOptResponse>("/swarm_expl/pair_opt_res_send", 10);
  opt_res_sub_ = nh.subscribe("/swarm_expl/pair_opt_res_recv", 100,
      &FameExplorationFSM::optResMsgCallback, this, ros::TransportHints().tcpNoDelay());

  swarm_traj_pub_ = nh.advertise<bspline::Bspline>("/planning/swarm_traj_send", 100);
  swarm_traj_sub_ =
      nh.subscribe("/planning/swarm_traj_recv", 100, &FameExplorationFSM::swarmTrajCallback, this);
  swarm_traj_timer_ =
      nh.createTimer(ros::Duration(0.1), &FameExplorationFSM::swarmTrajTimerCallback, this);

  hgrid_pub_ = nh.advertise<exploration_manager::HGrid>("/swarm_expl/hgrid_send", 10);
  grid_tour_pub_ = nh.advertise<exploration_manager::GridTour>("/swarm_expl/grid_tour_send", 10);
}

int FameExplorationFSM::getId() {
  return expl_manager_->ep_->drone_id_;
}

void FameExplorationFSM::sendStopMsg(int code) {
  std_msgs::Int32 msg;
  msg.data = code;

  const size_t kNumAttempts = 5;
  for (size_t i = 0; i < kNumAttempts; ++i) {
    stop_pub_.publish(msg);
    ros::Duration(0.01).sleep();
  }
}

void FameExplorationFSM::sendEmergencyMsg(bool emergency) {
  std_msgs::Bool msg;
  msg.data = emergency;
  emergency_handler_pub_.publish(msg);
}

void FameExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  ROS_INFO_STREAM_THROTTLE(
      1.0, "[FSM]: Drone " << getId() << " state: " << fame_data_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fame_data_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom");
        return;
      }
      if ((ros::Time::now() - fame_data_->fsm_init_time_).toSec() < 2.0) {
        ROS_WARN_THROTTLE(1.0, "wait for init");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");

      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
        fame_data_->trigger_ = true;
        cout << "Triggered!" << endl;
        fame_data_->start_pos_ = fame_data_->odom_pos_;
        // ROS_WARN_STREAM("Start expl pos: " << fame_data_->start_pos_.transpose());

        if (expl_manager_->updateFrontierStruct(fame_data_->odom_pos_, fame_data_->odom_yaw_, fame_data_->odom_vel_) != 0) {
        transitState(PLAN_TRAJ, "triggerCallback");
  } else
    transitState(FINISH, "triggerCallback");
      break;
    }

    case FINISH: {
      sendStopMsg(1);
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case IDLE: {
      double check_interval = (ros::Time::now() - fame_data_->last_check_frontier_time_).toSec();

      // Check: if we don't have any frontier, then stop
      if (expl_manager_->updateFrontierStruct(fame_data_->odom_pos_, fame_data_->odom_yaw_, fame_data_->odom_vel_) <=
          1) {
        ROS_WARN_THROTTLE(1., "No frontiers for agent %d", getId());
        sendStopMsg(1);
        break;
      }

      if (check_interval > 100.0) {
        ROS_WARN("Go back to (0,0,1)");

        expl_manager_->ed_->next_pos_ = fame_data_->start_pos_;
        expl_manager_->ed_->next_yaw_ = 0.0;

        fame_data_->go_back_ = true;
        transitState(PLAN_TRAJ, "FSM");
        // } else {
        //   fame_data_->last_check_frontier_time_ = ros::Time::now();
        // }
      }
      break;
    }

    case PLAN_TRAJ: {
      if (fame_data_->static_state_) {
        // Plan from static state (hover)
        fame_data_->start_pt_ = fame_data_->odom_pos_;
        fame_data_->start_vel_ = fame_data_->odom_vel_;
        fame_data_->start_acc_.setZero();
        fame_data_->start_yaw_ << fame_data_->odom_yaw_, 0, 0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fame_params_->replan_time_;
        fame_data_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fame_data_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fame_data_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fame_data_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fame_data_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fame_data_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }
      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res = callExplorationPlanner();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
        // Emergency
        num_fail_ = 0;
        sendEmergencyMsg(false);

      } else if (res == FAIL) {  // Keep trying to replan
        fame_data_->static_state_ = true;
        ROS_WARN_THROTTLE(1., "Plan fail (drone %d)", getId());
        // Check if we need to send a message
        // if (num_fail_ > 1000000000000) {
        //   sendEmergencyMsg(true);
        //   num_fail_ = 0;
        // } else {
        //   ++num_fail_;
        // }

      } else if (res == NO_GRID) {
        fame_data_->static_state_ = true;
        fame_data_->last_check_frontier_time_ = ros::Time::now();
        // ROS_WARN("No grid (drone %d)", getId());
        transitState(IDLE, "FSM");

        // Emergency
        num_fail_ = 0;
        sendEmergencyMsg(false);
      }
      visualize(1);
      // clearVisMarker();
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fame_data_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fame_data_->newest_traj_);
        fame_data_->static_state_ = false;

        // fame_data_->newest_traj_.drone_id = planner_manager_->swarm_traj_data_.drone_id_;
        fame_data_->newest_traj_.drone_id = expl_manager_->ep_->drone_id_;
        swarm_traj_pub_.publish(fame_data_->newest_traj_);

        thread vis_thread(&FameExplorationFSM::visualize, this, 2);
        vis_thread.detach();
        transitState(EXEC_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      auto tn = ros::Time::now();
      // Check whether replan is needed
      // ROS_WARN("Vector: x=%f, y=%f, z=%f", expl_manager_->ed_->next_pos_.x(), expl_manager_->ed_->next_pos_.y(), expl_manager_->ed_->next_pos_.z());
      if (fame_data_->idle)
      {
      geometry_msgs::Pose pose =  mrsFly(expl_manager_->ed_->next_pos_);
      pose_pub_.publish(pose);

      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (tn - info->start_time_).toSec();

      if (!fame_data_->go_back_) {
        bool need_replan = false;
        if (t_cur > fame_params_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
          // ROS_WARN("Replan: cluster covered=====================================");
          need_replan = true;
        } else if (info->duration_ - t_cur < fame_params_->replan_thresh1_) {
          // Replan if traj is almost fully executed
          // ROS_WARN("Replan: traj fully executed=================================");
          need_replan = true;
        } else if (t_cur > fame_params_->replan_thresh3_) {
          // Replan after some time
          // ROS_WARN("Replan: periodic call=======================================");
          need_replan = true;
        }

        if (need_replan) {
          if (expl_manager_->updateFrontierStruct(fame_data_->odom_pos_, fame_data_->odom_yaw_, fame_data_->odom_vel_) !=
              0) {
            // Update frontier and plan new motion
            thread vis_thread(&FameExplorationFSM::visualize, this, 1);
            vis_thread.detach();
            transitState(PLAN_TRAJ, "FSM");
          } else {
            // No frontier detected, finish exploration
            fame_data_->last_check_frontier_time_ = ros::Time::now();
            transitState(IDLE, "FSM");
            ROS_WARN_THROTTLE(1., "Idle since no frontier is detected");
            fame_data_->static_state_ = true;
            replan_pub_.publish(std_msgs::Empty());
            sendStopMsg(1);
          }
          // clearVisMarker();
          visualize(1);
        }
      } else {
        // Check if reach goal
        auto pos = info->position_traj_.evaluateDeBoorT(t_cur);
        if ((pos - expl_manager_->ed_->next_pos_).norm() < 1.0) {
          replan_pub_.publish(std_msgs::Empty());
          clearVisMarker();
          transitState(FINISH, "FSM");
          return;
        }
        if (t_cur > fame_params_->replan_thresh3_ || info->duration_ - t_cur < fame_params_->replan_thresh1_) {
          // Replan for going back
          replan_pub_.publish(std_msgs::Empty());
          transitState(PLAN_TRAJ, "FSM");
          thread vis_thread(&FameExplorationFSM::visualize, this, 1);
          vis_thread.detach();
        }
      }

      break;
      }
    }
  }
}

int FameExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fame_params_->replan_time_);

  int res;
  if (fame_data_->avoid_collision_ || fame_data_->go_back_) {  // Only replan trajectory
    res = expl_manager_->planTrajToView(fame_data_->start_pt_, fame_data_->start_vel_, fame_data_->start_acc_,
        fame_data_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
    fame_data_->avoid_collision_ = false;
    ROS_WARN("res at planTrajToView %d",res);
  } else {  // Do full planning normally
    res = expl_manager_->planExploreMotion(
        fame_data_->start_pt_, fame_data_->start_vel_, fame_data_->start_acc_, fame_data_->start_yaw_);
      ROS_WARN("res at planExploreMotion %d",res);
  }
  // ROS_WARN("res at 350 %d",res);
  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    


    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      mrs_msgs::Reference pose;
      pose.position.x = pos_pts(i, 0);
      pose.position.y = pos_pts(i, 1);
      pose.position.z = pos_pts(i, 2);
      pose.heading    = 0;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fame_data_->newest_traj_ = bspline;
  }
  // ROS_WARN("res at 387 %d",res);
  return res;
}

void FameExplorationFSM::visualize(int content) {
  // content 1: frontier; 2 paths & trajs
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
    double a = (drone_id - 1) / double(num + 1);
    double b = 1 / double(num + 1);
    return a + b * double(id) / ed_ptr->frontiers_.size();
  };

  if (content == 1) {
    // Draw frontier
    auto res = expl_manager_->sdf_map_->getResolution();
    static int last_ftr_num = 0;
    for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
      auto color = visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4);
      visualization_->drawCubes(ed_ptr->frontiers_[i], res, color, "frontier", i, 4);

    }
    for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
      visualization_->drawCubes({}, res, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    }
    last_ftr_num = ed_ptr->frontiers_.size();

    // Draw labeled frontier
    static size_t last_ftr_num_labeled = 0;
    for (size_t i = 0; i < ed_ptr->labeled_frontiers_.size(); ++i) {
      Eigen::Vector4d color(0, 0, 0, 0.5);
      if (ed_ptr->labeled_frontiers_[i].first == 0) {
        color[0] = 1.;
      } else if (ed_ptr->labeled_frontiers_[i].first == LABEL::FRONTIER) {
        color[1] = 1.;
      } else {
        color[2] = 1.;
      }
      visualization_->drawCubes(
          ed_ptr->labeled_frontiers_[i].second, res, color, "labeled_frontier", i, 7);
    }
    for (size_t i = ed_ptr->labeled_frontiers_.size(); i < last_ftr_num_labeled; ++i) {
      visualization_->drawCubes({}, res, Eigen::Vector4d(0, 0, 0, 1), "labeled_frontier", i, 7);
    }
    last_ftr_num_labeled = ed_ptr->labeled_frontiers_.size();

    // Draw frontiers in front
    static size_t last_ftr_num_infront = 0;
    for (size_t i = 0; i < ed_ptr->infront_frontiers_.size(); ++i) {
      Eigen::Vector4d color(1, 1, 0, 0.5);
      visualization_->drawCubes(
          ed_ptr->infront_frontiers_[i].second, res, color, "infront_frontiers", i, 10);
    }
    for (size_t i = ed_ptr->infront_frontiers_.size(); i < last_ftr_num_labeled; ++i) {
      visualization_->drawCubes({}, res, Eigen::Vector4d(0, 0, 0, 1), "infront_frontiers", i, 10);
    }
    last_ftr_num_infront = ed_ptr->infront_frontiers_.size();

    // Publish role
    const std::string role_str = roleToString(expl_manager_->role_);
    visualization_->drawText(fame_data_->odom_pos_ - Eigen::Vector3d(1., 0., 0.), role_str, 0.4,
        Eigen::Vector4d::Ones(), "role", 0, 8);

    // Publish current goal
    auto color =
        PlanningVisualization::getColor((getId() - 1) / double(expl_manager_->ep_->drone_num_));
    visualization_->drawArrow(expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_,
        Eigen::Vector3d(0.75, 0.3, 0.75), color, "goal", 0, 9);

    // Publish trails tour
    if (expl_manager_->role_ == ROLE::GARBAGE_COLLECTOR) {
      std::vector<Eigen::Vector3d> tour;
      for (const auto& p : expl_manager_->ed_->trails_tour_) tour.push_back(p.first);
      visualization_->drawLines(tour, 0.07, color, "trails_tour", 0, 11);
    } else {
      visualization_->drawLines({}, 0.07, Eigen::Vector4d::Zero(), "trails_tour", 0, 11);
    }

  } else if (content == 2) {


    if (expl_manager_->ep_->drone_id_ == 1) {
      vector<Eigen::Vector3d> pts1, pts2;
      expl_manager_->hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      expl_manager_->hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (int i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(
            Eigen::Vector3d(0, 0, 0), string(""), 1, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      last_text_num = pts.size();

    }

    auto grid_tour = expl_manager_->ed_->grid_tour_;
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        "grid_tour", 0, 6);

    // Publish grid tour to ground node
    exploration_manager::GridTour tour;
    for (int i = 0; i < grid_tour.size(); ++i) {
      geometry_msgs::Point point;
      point.x = grid_tour[i][0];
      point.y = grid_tour[i][1];
      point.z = grid_tour[i][2];
      tour.points.push_back(point);
    }
    tour.drone_id = expl_manager_->ep_->drone_id_;
    tour.stamp = ros::Time::now().toSec();
    grid_tour_pub_.publish(tour);

    visualization_->drawBspline(info->position_traj_, 0.1,
        PlanningVisualization::getColor(
            (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
        false, 0.15, Vector4d(1, 1, 0, 1));
  }
}

geometry_msgs::Pose FameExplorationFSM::mrsFly(Vector3d& pos) {
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = pos.x();
    pose_msg.position.y = pos.y();
    pose_msg.position.z = pos.z();
    pose_msg.orientation.w    = 1.0;
    pose_msg.orientation.x    = 0.0;
    pose_msg.orientation.y    = 0.0;
    pose_msg.orientation.z    = 0.0;
  return pose_msg;
}

void FameExplorationFSM::clearVisMarker() {
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
  }
  for (int i = 0; i < 10; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "lebeled_frontier", i, 7);
  }
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "frontier_tour", 0, 6);
  visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "grid_tour", 0, 6);
  visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

}

void FameExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  if (state_ == INIT) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;

    auto getColorVal = [&](const int& id, const int& num, const int& drone_id) {
      double a = (drone_id - 1) / double(num + 1);
      double b = 1 / double(num + 1);
      return a + b * double(id) / ed->frontiers_.size();
    };


    expl_manager_->updateFrontierStruct(fame_data_->odom_pos_, fame_data_->odom_yaw_, fame_data_->odom_vel_);


    // Draw frontier and bounding box
    auto res = expl_manager_->sdf_map_->getResolution();
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      auto color = visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4);
      visualization_->drawCubes(ed->frontiers_[i], res, color, "frontier", i, 4);
      // getColorVal(i, expl_manager_->ep_->drone_num_, expl_manager_->ep_->drone_id_)
      // double(i) / ed->frontiers_.size()
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      //   color, "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, res, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      //   "frontier_boxes", i, 4);
    }

    // Draw labeled frontier
    for (size_t i = 0; i < ed->labeled_frontiers_.size(); ++i) {
      Eigen::Vector4d color(0, 0, 0, 0.5);
      if (ed->labeled_frontiers_[i].first == LABEL::TRAIL) {
        color[0] = 1.;
      } else if (ed->labeled_frontiers_[i].first == LABEL::FRONTIER) {
        color[1] = 1.;
      } else {
        color[2] = 1.;
      }
      visualization_->drawCubes(
          ed->labeled_frontiers_[i].second, res, color, "labeled_frontier", i, 7);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, res, Vector4d(0, 0, 0, 1), "labeled_frontier", i, 7);
    }

    visualize(2);
  }
}

void FameExplorationFSM::heartbitCallback(const ros::TimerEvent& e) {
  heartbit_pub_.publish(std_msgs::Empty());
}

void FameExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      // ROS_WARN("Replan: collision detected==================================");
      fame_data_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "safetyCallback");
    }

    static auto time_check = ros::Time::now();
    if (expl_manager_->sdf_map_->getOccupancy(fame_data_->odom_pos_) != SDFMap::OCCUPANCY::FREE) {
      if ((ros::Time::now() - time_check).toSec() > 20.) {
        sendStopMsg(-1);
      }
    } else {
      time_check = ros::Time::now();
    }
  }
}

void FameExplorationFSM::diagnosticCallback(const mrs_modules_msgs::OctomapPlannerDiagnostics& msg) {
  fame_data_->idle = msg.idle;
}

void FameExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fame_data_->odom_pos_(0) = msg->pose.pose.position.x;
  fame_data_->odom_pos_(1) = msg->pose.pose.position.y;
  fame_data_->odom_pos_(2) = msg->pose.pose.position.z;

  fame_data_->odom_vel_(0) = msg->twist.twist.linear.x;
  fame_data_->odom_vel_(1) = msg->twist.twist.linear.y;
  fame_data_->odom_vel_(2) = msg->twist.twist.linear.z;

  fame_data_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fame_data_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fame_data_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fame_data_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fame_data_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fame_data_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  if (!fame_data_->have_odom_) {
    fame_data_->have_odom_ = true;
    fame_data_->fsm_init_time_ = ros::Time::now();
  }
}

void FameExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  ROS_INFO_STREAM("[" + pos_call + "]: Drone "
                  << getId()
                  << " from " + fame_data_->state_str_[pre_s] + " to " + fame_data_->state_str_[int(new_state)]);
}

void FameExplorationFSM::droneStateTimerCallback(const ros::TimerEvent& e) {
  // Broadcast own state periodically
  exploration_manager::DroneState msg;
  msg.drone_id = getId();

  auto& state = expl_manager_->ed_->swarm_state_[msg.drone_id - 1];

  if (fame_data_->static_state_) {
    state.pos_ = fame_data_->odom_pos_;
    state.vel_ = fame_data_->odom_vel_;
    state.yaw_ = fame_data_->odom_yaw_;
  } else {
    LocalTrajData* info = &planner_manager_->local_data_;
    double t_r = (ros::Time::now() - info->start_time_).toSec();
    state.pos_ = info->position_traj_.evaluateDeBoorT(t_r);
    state.vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
    state.yaw_ = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
  }
  state.stamp_ = ros::Time::now().toSec();
  msg.pos = { float(state.pos_[0]), float(state.pos_[1]), float(state.pos_[2]) };
  msg.vel = { float(state.vel_[0]), float(state.vel_[1]), float(state.vel_[2]) };
  msg.yaw = state.yaw_;
  for (auto id : state.grid_ids_) msg.grid_ids.push_back(id);
  msg.recent_attempt_time = state.recent_attempt_time_;
  msg.stamp = state.stamp_;
  msg.goal_posit = eigenToGeometryMsg(expl_manager_->ed_->next_pos_);
  msg.role = int(expl_manager_->role_);

  drone_state_pub_.publish(msg);
}

void FameExplorationFSM::droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg) {
  // Update other drones' states
  if (msg->drone_id == getId()) return;

  // Skip update if collaboration is inactive
  if (!coll_assigner_->isActive()) return;

  // Simulate swarm communication loss
  const Eigen::Vector3d msg_pos(msg->pos[0], msg->pos[1], msg->pos[2]);
  if ((msg_pos - fame_data_->odom_pos_).norm() > fame_params_->communication_range_) {
    return;
  }

  auto& drone_state = expl_manager_->ed_->swarm_state_[msg->drone_id - 1];
  if (drone_state.stamp_ + 1e-4 >= msg->stamp) return;  // Avoid unordered msg

  drone_state.pos_ = Eigen::Vector3d(msg->pos[0], msg->pos[1], msg->pos[2]);
  drone_state.vel_ = Eigen::Vector3d(msg->vel[0], msg->vel[1], msg->vel[2]);
  drone_state.yaw_ = msg->yaw;
  drone_state.grid_ids_.clear();
  for (auto id : msg->grid_ids) drone_state.grid_ids_.push_back(id);
  drone_state.stamp_ = msg->stamp;
  drone_state.recent_attempt_time_ = msg->recent_attempt_time;
  drone_state.goal_pos_ = geometryMsgToEigen(msg->goal_posit);
  drone_state.role_ = ROLE(msg->role);

  // std::cout << "Drone " << getId() << " get drone " << int(msg->drone_id) << "'s state" <<
  // std::endl; std::cout << drone_state.pos_.transpose() << std::endl;
}

void FameExplorationFSM::optTimerCallback(const ros::TimerEvent& e) {
  if (state_ == INIT) return;

  // Select nearby drone not interacting with recently
  auto& states = expl_manager_->ed_->swarm_state_;
  auto& state1 = states[getId() - 1];
  // bool urgent = (state1.grid_ids_.size() <= 1 /* && !state1.grid_ids_.empty() */);
  bool urgent = state1.grid_ids_.empty();
  auto tn = ros::Time::now().toSec();

  // Avoid frequent attempt
  if (tn - state1.recent_attempt_time_ < fame_params_->attempt_interval_) return;

  int select_id = -1;
  double max_interval = -1.0;
  for (int i = 0; i < states.size(); ++i) {
    if (i + 1 <= getId()) continue;
    // Check if have communication recently
    // or the drone just experience another opt
    // or the drone is interacted with recently /* !urgent &&  */
    if (tn - states[i].stamp_ > 0.2) continue;
    if (tn - states[i].recent_attempt_time_ < fame_params_->attempt_interval_) continue;
    if (tn - states[i].recent_interact_time_ < fame_params_->pair_opt_interval_) continue;

    double interval = tn - states[i].recent_interact_time_;
    if (interval <= max_interval) continue;
    select_id = i + 1;
    max_interval = interval;
  }
  if (select_id == -1) return;

  std::cout << "\nSelect: " << select_id << std::endl;
  // ROS_WARN("Pair opt %d & %d", getId(), select_id);

  // Do pairwise optimization with selected drone, allocate goal poses
  auto ego_goal = expl_manager_->ed_->next_pos_;
  auto& state2 = states[select_id - 1];
  auto other_goal = state2.goal_pos_;

  auto t1 = ros::Time::now();
  Eigen::Vector3d ego_opt, other_opt;
  const bool res = coll_assigner_->optimizePositions(ego_goal, other_goal, ego_opt, other_opt);
  double alloc_time = (ros::Time::now() - t1).toSec();
  if (!res) {
    ROS_ERROR("Opt failed - keep same assignment");
  }

  // Send the result to selected drone and wait for confirmation
  exploration_manager::PairOpt opt;
  opt.from_drone_id = getId();
  opt.to_drone_id = select_id;
  // opt.msg_type = 1;
  opt.stamp = tn;
  // for (auto id : ego_ids) opt.ego_ids.push_back(id);
  // for (auto id : other_ids) opt.other_ids.push_back(id);

  for (int i = 0; i < fame_params_->repeat_send_num_; ++i) opt_pub_.publish(opt);

  // ROS_WARN("Drone %d send opt request to %d, pair opt t: %lf, allocate t: %lf", getId(),
  // select_id,
  //     ros::Time::now().toSec() - tn, alloc_time);

  // Reserve the result and wait...
  auto ed = expl_manager_->ed_;
  // ed->ego_ids_ = ego_ids;
  // ed->other_ids_ = other_ids;
  ed->next_pos_ = ego_opt;
  ed->swarm_state_[select_id - 1].goal_pos_ = other_opt;
  ed->pair_opt_stamp_ = opt.stamp;
  ed->wait_response_ = true;
  state1.recent_attempt_time_ = tn;
}

void FameExplorationFSM::findUnallocated(const vector<int>& actives, vector<int>& missed) {
  // Create map of all active
  unordered_map<int, char> active_map;
  for (auto ativ : actives) {
    active_map[ativ] = 1;
  }

  // Remove allocated ones
  for (auto state : expl_manager_->ed_->swarm_state_) {
    for (auto id : state.grid_ids_) {
      if (active_map.find(id) != active_map.end()) {
        active_map.erase(id);
      } else {
        // ROS_ERROR("Inactive grid %d is allocated.", id);
      }
    }
  }

  missed.clear();
  for (auto p : active_map) {
    missed.push_back(p.first);
  }
}

void FameExplorationFSM::optMsgCallback(const exploration_manager::PairOptConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] + 1e-4) return;
  expl_manager_->ed_->pair_opt_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto& state1 = expl_manager_->ed_->swarm_state_[msg->from_drone_id - 1];
  auto& state2 = expl_manager_->ed_->swarm_state_[getId() - 1];

  // auto tn = ros::Time::now().toSec();
  exploration_manager::PairOptResponse response;
  response.from_drone_id = msg->to_drone_id;
  response.to_drone_id = msg->from_drone_id;
  response.stamp = msg->stamp;  // reply with the same stamp for verificaiton

  if (msg->stamp - state2.recent_attempt_time_ < fame_params_->attempt_interval_) {
    // Just made another pair opt attempt, should reject this attempt to avoid frequent changes
    // ROS_WARN("Reject frequent attempt");
    response.status = 2;
  } else {
    // No opt attempt recently, and the grid info between drones are consistent, the pair opt
    // request can be accepted
    response.status = 1;

    // Update from the opt result
    state1.grid_ids_.clear();
    state2.grid_ids_.clear();
    for (auto id : msg->ego_ids) state1.grid_ids_.push_back(id);
    for (auto id : msg->other_ids) state2.grid_ids_.push_back(id);

    state1.goal_pos_ = geometryMsgToEigen(msg->ego_posit);
    state2.goal_pos_ = geometryMsgToEigen(msg->other_posit);

    state1.recent_interact_time_ = msg->stamp;
    state2.recent_attempt_time_ = ros::Time::now().toSec();
    expl_manager_->ed_->reallocated_ = true;

    if (state_ == IDLE && !state2.grid_ids_.empty()) {
      transitState(PLAN_TRAJ, "optMsgCallback");
      // ROS_WARN("Restart after opt!");
    }

    // if (!check_consistency(tmp1, tmp2)) {
    //   response.status = 2;
    //   ROS_WARN("Inconsistent grid info, reject pair opt");
    // } else {
    // }
  }
  for (int i = 0; i < fame_params_->repeat_send_num_; ++i) opt_res_pub_.publish(response);
}

void FameExplorationFSM::optResMsgCallback(
    const exploration_manager::PairOptResponseConstPtr& msg) {
  if (msg->from_drone_id == getId() || msg->to_drone_id != getId()) return;

  // Check stamp to avoid unordered/repeated msg
  if (msg->stamp <= expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] + 1e-4) return;
  expl_manager_->ed_->pair_opt_res_stamps_[msg->from_drone_id - 1] = msg->stamp;

  auto ed = expl_manager_->ed_;
  // Verify the consistency of pair opt via time stamp
  if (!ed->wait_response_ || fabs(ed->pair_opt_stamp_ - msg->stamp) > 1e-5) return;

  ed->wait_response_ = false;
  // ROS_WARN("get response %d", int(msg->status));

  if (msg->status != 1) return;  // Receive 1 for valid opt

  auto& state1 = ed->swarm_state_[getId() - 1];
  auto& state2 = ed->swarm_state_[msg->from_drone_id - 1];
  state1.grid_ids_ = ed->ego_ids_;
  state2.grid_ids_ = ed->other_ids_;
  // FIXME goal here?
  state2.recent_interact_time_ = ros::Time::now().toSec();
  ed->reallocated_ = true;

  if (state_ == IDLE && !state1.grid_ids_.empty()) {
    transitState(PLAN_TRAJ, "optResMsgCallback");
    // ROS_WARN("Restart after opt!");
  }
}

void FameExplorationFSM::swarmTrajCallback(const bspline::BsplineConstPtr& msg) {
  // Get newest trajs from other drones, for inter-drone collision avoidance
  auto& sdat = planner_manager_->swarm_traj_data_;

  // Ignore self trajectory
  if (msg->drone_id == sdat.drone_id_) return;

  // Ignore outdated trajectory
  if (sdat.receive_flags_[msg->drone_id - 1] == true &&
      msg->start_time.toSec() <= sdat.swarm_trajs_[msg->drone_id - 1].start_time_ + 1e-3)
    return;

  // Convert the msg to B-spline
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }


  sdat.swarm_trajs_[msg->drone_id - 1].setUniformBspline(pos_pts, msg->order, 0.1);
  sdat.swarm_trajs_[msg->drone_id - 1].setKnot(knots);
  sdat.swarm_trajs_[msg->drone_id - 1].start_time_ = msg->start_time.toSec();
  sdat.receive_flags_[msg->drone_id - 1] = true;

  if (state_ == EXEC_TRAJ) {
    // Check collision with received trajectory
    if (!planner_manager_->checkSwarmCollision(msg->drone_id)) {
      ROS_ERROR("Drone %d collide with drone %d.", sdat.drone_id_, msg->drone_id);
      fame_data_->avoid_collision_ = true;
      transitState(PLAN_TRAJ, "swarmTrajCallback");
    }
  }
}

void FameExplorationFSM::swarmTrajTimerCallback(const ros::TimerEvent& e) {
  // Broadcast newest traj of this drone to others
  if (state_ == EXEC_TRAJ) {
    swarm_traj_pub_.publish(fame_data_->newest_traj_);

  } else if (state_ == INIT) {
    // Publish a virtual traj at current pose, to avoid collision
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = ros::Time::now();
    bspline.traj_id = planner_manager_->local_data_.traj_id_;

    Eigen::MatrixXd pos_pts(4, 3);
    for (int i = 0; i < 4; ++i) pos_pts.row(i) = fame_data_->odom_pos_.transpose();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    NonUniformBspline tmp(pos_pts, planner_manager_->pp_.bspline_degree_, 1.0);
    Eigen::VectorXd knots = tmp.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    bspline.drone_id = expl_manager_->ep_->drone_id_;
    swarm_traj_pub_.publish(bspline);
  }
}

}  // namespace fast_planner
