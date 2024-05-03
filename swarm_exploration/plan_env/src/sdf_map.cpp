#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
#include <plan_env/multi_map_manager.h>
#include <plan_env/raycast.h>

namespace fast_planner {
SDFMap::SDFMap() {
}

SDFMap::~SDFMap() {
}

void SDFMap::initMap(ros::NodeHandle& nh) {
  map_param_.reset(new MapParam);
  map_data_.reset(new MapData);
  map_ros_.reset(new MapROS);
  multi_map_.reset(new MultiMapManager);

  // Params of map properties
  double x_size, y_size, z_size;
  nh.param("sdf_map/resolution", map_param_->resolution_, -1.0);
  nh.param("sdf_map/map_size_x", x_size, -1.0);
  nh.param("sdf_map/map_size_y", y_size, -1.0);
  nh.param("sdf_map/map_size_z", z_size, -1.0);
  nh.param("sdf_map/obstacles_inflation", map_param_->obstacles_inflation_, -1.0);
  nh.param("sdf_map/local_bound_inflate", map_param_->local_bound_inflate_, 1.0);
  nh.param("sdf_map/local_map_margin", map_param_->local_map_margin_, 1);
  nh.param("sdf_map/ground_height", map_param_->ground_height_, 1.0);
  nh.param("sdf_map/default_dist", map_param_->default_dist_, 5.0);
  nh.param("sdf_map/optimistic", map_param_->optimistic_, true);
  nh.param("sdf_map/signed_dist", map_param_->signed_dist_, false);
  nh.param("sdf_map/no_drone_1", map_param_->no_drone_1_, false);

  map_param_->local_bound_inflate_ = max(map_param_->resolution_, map_param_->local_bound_inflate_);
  map_param_->resolution_inv_ = 1 / map_param_->resolution_;
  map_param_->map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, map_param_->ground_height_);
  map_param_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
  for (int i = 0; i < 3; ++i) map_param_->map_voxel_num_(i) = ceil(map_param_->map_size_(i) / map_param_->resolution_);
  map_param_->map_min_boundary_ = map_param_->map_origin_;
  map_param_->map_max_boundary_ = map_param_->map_origin_ + map_param_->map_size_;

  // Params of raycasting-based fusion
  nh.param("sdf_map/p_hit", map_param_->p_hit_, 0.70);
  nh.param("sdf_map/p_miss", map_param_->p_miss_, 0.35);
  nh.param("sdf_map/p_min", map_param_->p_min_, 0.12);
  nh.param("sdf_map/p_max", map_param_->p_max_, 0.97);
  nh.param("sdf_map/p_occ", map_param_->p_occ_, 0.80);
  nh.param("sdf_map/max_ray_length", map_param_->max_ray_length_, -0.1);
  nh.param("sdf_map/virtual_ceil_height", map_param_->virtual_ceil_height_, -0.1);

  auto logit = [](const double& x) { return log(x / (1 - x)); };
  map_param_->prob_hit_log_ = logit(map_param_->p_hit_);
  map_param_->prob_miss_log_ = logit(map_param_->p_miss_);
  map_param_->clamp_min_log_ = logit(map_param_->p_min_);
  map_param_->clamp_max_log_ = logit(map_param_->p_max_);
  map_param_->min_occupancy_log_ = logit(map_param_->p_occ_);
  map_param_->unknown_flag_ = 0.01;
  cout << "hit: " << map_param_->prob_hit_log_ << ", miss: " << map_param_->prob_miss_log_
       << ", min: " << map_param_->clamp_min_log_ << ", max: " << map_param_->clamp_max_log_
       << ", thresh: " << map_param_->min_occupancy_log_ << endl;

  // Initialize data buffer of map
  int buffer_size = map_param_->map_voxel_num_(0) * map_param_->map_voxel_num_(1) * map_param_->map_voxel_num_(2);
  map_data_->occupancy_buffer_ = vector<double>(buffer_size, map_param_->clamp_min_log_ - map_param_->unknown_flag_);
  map_data_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  map_data_->distance_buffer_neg_ = vector<double>(buffer_size, map_param_->default_dist_);
  map_data_->distance_buffer_ = vector<double>(buffer_size, map_param_->default_dist_);
  map_data_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
  map_data_->count_hit_ = vector<short>(buffer_size, 0);
  map_data_->count_miss_ = vector<short>(buffer_size, 0);
  map_data_->flag_rayend_ = vector<char>(buffer_size, -1);
  map_data_->flag_visited_ = vector<char>(buffer_size, -1);
  map_data_->tmp_buffer1_ = vector<double>(buffer_size, 0);
  map_data_->tmp_buffer2_ = vector<double>(buffer_size, 0);
  map_data_->raycast_num_ = 0;
  map_data_->reset_updated_box_ = true;

  // Try retriving bounding box of map, set box to map size if not specified
  vector<string> axis = { "x", "y", "z" };
  for (int i = 0; i < 3; ++i) {
    nh.param("sdf_map/box_min_" + axis[i], map_param_->box_mind_[i], map_param_->map_min_boundary_[i]);
    nh.param("sdf_map/box_max_" + axis[i], map_param_->box_maxd_[i], map_param_->map_max_boundary_[i]);
  }
  posToIndex(map_param_->box_mind_, map_param_->box_min_);
  posToIndex(map_param_->box_maxd_, map_param_->box_max_);

  // Initialize ROS wrapper
  map_ros_->setMap(this);
  map_ros_->node_ = nh;
  map_ros_->init();

  multi_map_->setMap(this);
  multi_map_->node_ = nh;
  multi_map_->init();

  caster_.reset(new RayCaster);
  caster_->setParams(map_param_->resolution_, map_param_->map_origin_);

  for (int i = 0; i < 3; ++i) {
    map_data_->all_min_[i] = 1000000;
    map_data_->all_max_[i] = -1000000;
  }

  // Compute initial vertices, box and normals of map in current drone's frame
  map_param_->rot_sw_ << cos(0.0), -sin(0.0), 0, sin(0.0), cos(0.0), 0, 0, 0, 1;
  map_param_->trans_sw_ << 0, 0, 0;

  Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
  left_bottom = map_param_->box_mind_;
  right_top = map_param_->box_maxd_;

  left_top[0] = left_bottom[0];
  left_top[1] = right_top[1];
  left_top[2] = left_bottom[2];
  right_bottom[0] = right_top[0];
  right_bottom[1] = left_bottom[1];
  right_bottom[2] = left_bottom[2];
  right_top[2] = left_bottom[2];

  map_param_->vertices_ = { left_bottom, right_bottom, right_top, left_top };
  if (map_param_->use_swarm_tf_) {
    for (auto& vert : map_param_->vertices_) vert = map_param_->rot_sw_ * vert + map_param_->trans_sw_;
  }

  map_param_->vmin_ = map_param_->vmax_ = map_param_->vertices_[0];
  for (int j = 1; j < map_param_->vertices_.size(); ++j) {
    for (int k = 0; k < 2; ++k) {
      map_param_->vmin_[k] = min(map_param_->vmin_[k], map_param_->vertices_[j][k]);
      map_param_->vmax_[k] = max(map_param_->vmax_[k], map_param_->vertices_[j][k]);
    }
  }

  for (int j = 0; j < 4; ++j) {
    Eigen::Vector3d dir = (map_param_->vertices_[(j + 1) % 4] - map_param_->vertices_[j]).normalized();
    map_param_->normals_.push_back(dir);
  }
}

void SDFMap::resetBuffer() {
  resetBuffer(map_param_->map_min_boundary_, map_param_->map_max_boundary_);
  map_data_->local_bound_min_ = Eigen::Vector3i::Zero();
  map_data_->local_bound_max_ = map_param_->map_voxel_num_ - Eigen::Vector3i::Ones();
}

void SDFMap::resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        map_data_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        map_data_->distance_buffer_[toAddress(x, y, z)] = map_param_->default_dist_;
      }
}

template <typename F_get_val, typename F_set_val>
void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[map_param_->map_voxel_num_(dim)];
  double z[map_param_->map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void SDFMap::updateESDF3d() {
  Eigen::Vector3i min_esdf = map_data_->local_bound_min_;
  Eigen::Vector3i max_esdf = map_data_->local_bound_max_;

  if (map_param_->optimistic_) {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return map_data_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ?
                         0 :
                         std::numeric_limits<double>::max();
            },
            [&](int z, double val) { map_data_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  } else {
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              int adr = toAddress(x, y, z);
              return (map_data_->occupancy_buffer_inflate_[adr] == 1 ||
                         map_data_->occupancy_buffer_[adr] < map_param_->clamp_min_log_ - 1e-3) ?
                         0 :
                         std::numeric_limits<double>::max();
            },
            [&](int z, double val) { map_data_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
  }

  for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int y) { return map_data_->tmp_buffer1_[toAddress(x, y, z)]; },
          [&](int y, double val) { map_data_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
          max_esdf[1], 1);
    }
  for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
    for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
      fillESDF([&](int x) { return map_data_->tmp_buffer2_[toAddress(x, y, z)]; },
          [&](int x, double val) {
            map_data_->distance_buffer_[toAddress(x, y, z)] = map_param_->resolution_ * std::sqrt(val);
          },
          min_esdf[0], max_esdf[0], 0);
    }

  if (map_param_->signed_dist_) {
    // Compute negative distance
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return map_data_->occupancy_buffer_inflate_[x * map_param_->map_voxel_num_(1) *
                                                        map_param_->map_voxel_num_(2) +
                                                    y * map_param_->map_voxel_num_(2) + z] == 0 ?
                         0 :
                         std::numeric_limits<double>::max();
            },
            [&](int z, double val) { map_data_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return map_data_->tmp_buffer1_[toAddress(x, y, z)]; },
            [&](int y, double val) { map_data_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
            max_esdf[1], 1);
      }
    for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return map_data_->tmp_buffer2_[toAddress(x, y, z)]; },
            [&](int x, double val) {
              map_data_->distance_buffer_neg_[toAddress(x, y, z)] = map_param_->resolution_ * std::sqrt(val);
            },
            min_esdf[0], max_esdf[0], 0);
      }
    // Merge negative distance with positive
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {
          int idx = toAddress(x, y, z);
          if (map_data_->distance_buffer_neg_[idx] > 0.0)
            map_data_->distance_buffer_[idx] += (-map_data_->distance_buffer_neg_[idx] + map_param_->resolution_);
        }
  }
}

void SDFMap::setCacheOccupancy(const int& adr, const int& occ) {
  // Add to update list if first visited
  if (map_data_->count_hit_[adr] == 0 && map_data_->count_miss_[adr] == 0) map_data_->cache_voxel_.push(adr);

  if (occ == 0)
    map_data_->count_miss_[adr] = 1;
  else /* if (occ == 1) */
    map_data_->count_hit_[adr] += 1;

  // map_data_->count_hit_and_miss_[adr] += 1;
  // if (occ == 1)
  //   map_data_->count_hit_[adr] += 1;
  // if (map_data_->count_hit_and_miss_[adr] == 1)
  //   map_data_->cache_voxel_.push(adr);
}

void SDFMap::inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
    const Eigen::Vector3d& camera_pos) {
  ROS_INFO("point num: %d", points.size());
  if (point_num == 0) return;
  map_data_->raycast_num_ += 1;

  Eigen::Vector3d update_min = camera_pos;
  Eigen::Vector3d update_max = camera_pos;
  if (map_data_->reset_updated_box_) {
    map_data_->update_min_ = camera_pos;
    map_data_->update_max_ = camera_pos;
    map_data_->reset_updated_box_ = false;
  }

  Eigen::Vector3d pt_w, tmp;
  Eigen::Vector3i idx;
  int vox_adr;
  double length;
  for (int i = 0; i < point_num; ++i) {
    auto& pt = points.points[i];
    pt_w << pt.x, pt.y, pt.z;
    int tmp_flag;
    // Set flag for projected point
    if (!isInMap(pt_w)) {
      // Find closest point in map and set free
      pt_w = closetPointInMap(pt_w, camera_pos);
      length = (pt_w - camera_pos).norm();
      if (length > map_param_->max_ray_length_)
        pt_w = (pt_w - camera_pos) / length * map_param_->max_ray_length_ + camera_pos;
      if (pt_w[2] < 0.2) continue;
      tmp_flag = 0;
    } else {
      length = (pt_w - camera_pos).norm();
      if (length > map_param_->max_ray_length_) {
        pt_w = (pt_w - camera_pos) / length * map_param_->max_ray_length_ + camera_pos;
        if (pt_w[2] < 0.2) continue;
        tmp_flag = 0;
      } else
        tmp_flag = 1;
    }
    posToIndex(pt_w, idx);
    vox_adr = toAddress(idx);
    setCacheOccupancy(vox_adr, tmp_flag);

    for (int k = 0; k < 3; ++k) {
      update_min[k] = min(update_min[k], pt_w[k]);
      update_max[k] = max(update_max[k], pt_w[k]);
    }
    // Raycasting between camera center and point
    if (map_data_->flag_rayend_[vox_adr] == map_data_->raycast_num_)
      continue;
    else
      map_data_->flag_rayend_[vox_adr] = map_data_->raycast_num_;

    caster_->input(pt_w, camera_pos);
    caster_->nextId(idx);
    while (caster_->nextId(idx)) setCacheOccupancy(toAddress(idx), 0);
  }

  Eigen::Vector3d bound_inf(map_param_->local_bound_inflate_, map_param_->local_bound_inflate_, 0);
  posToIndex(update_max + bound_inf, map_data_->local_bound_max_);
  posToIndex(update_min - bound_inf, map_data_->local_bound_min_);
  boundIndex(map_data_->local_bound_min_);
  boundIndex(map_data_->local_bound_max_);
  map_ros_->local_updated_ = true;

  // Bounding box for subsequent updating and overall updateing
  for (int k = 0; k < 3; ++k) {
    map_data_->update_min_[k] = min(update_min[k], map_data_->update_min_[k]);
    map_data_->update_max_[k] = max(update_max[k], map_data_->update_max_[k]);
    map_data_->all_min_[k] = min(update_min[k], map_data_->all_min_[k]);
    map_data_->all_max_[k] = max(update_max[k], map_data_->all_max_[k]);
  }

  // Update map data
  vector<uint32_t> new_voxel_ids_;
  while (!map_data_->cache_voxel_.empty()) {
    int adr = map_data_->cache_voxel_.front();
    map_data_->cache_voxel_.pop();

    // FIXME Why does this happen?
    if (adr >= map_data_->count_hit_.size()) continue;

    double log_odds_update =
        map_data_->count_hit_[adr] >= map_data_->count_miss_[adr] ? map_param_->prob_hit_log_ : map_param_->prob_miss_log_;
    map_data_->count_hit_[adr] = map_data_->count_miss_[adr] = 0;
    if (map_data_->occupancy_buffer_[adr] < map_param_->clamp_min_log_ - 1e-3) {
      map_data_->occupancy_buffer_[adr] = map_param_->min_occupancy_log_;
      new_voxel_ids_.push_back(adr);
    }
    map_data_->occupancy_buffer_[adr] =
        std::min(std::max(map_data_->occupancy_buffer_[adr] + log_odds_update, map_param_->clamp_min_log_),
            map_param_->clamp_max_log_);
  }

  multi_map_->updateMapChunk(new_voxel_ids_);
}

Eigen::Vector3d SDFMap::closetPointInMap(
    const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = map_param_->map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = map_param_->map_min_boundary_ - camera_pt;
  double min_t = 1000000;
  for (int i = 0; i < 3; ++i) {
    if (fabs(diff[i]) > 0) {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t) min_t = t1;
      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t) min_t = t2;
    }
  }
  return camera_pt + (min_t - 1e-3) * diff;
}

void SDFMap::clearAndInflateLocalMap() {
  // /*clear outside local*/
  // const int vec_margin = 5;

  // Eigen::Vector3i min_cut = map_data_->local_bound_min_ -
  //     Eigen::Vector3i(map_param_->local_map_margin_, map_param_->local_map_margin_,
  //     map_param_->local_map_margin_);
  // Eigen::Vector3i max_cut = map_data_->local_bound_max_ +
  //     Eigen::Vector3i(map_param_->local_map_margin_, map_param_->local_map_margin_,
  //     map_param_->local_map_margin_);
  // boundIndex(min_cut);
  // boundIndex(max_cut);

  // Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin,
  // vec_margin); Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); boundIndex(min_cut_m); boundIndex(max_cut_m);

  // // clear data outside the local range

  // for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
  //   for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

  //     for (int z = min_cut_m(2); z < min_cut(2); ++z) {
  //       int idx                       = toAddress(x, y, z);
  //       map_data_->occupancy_buffer_[idx]    = map_param_->clamp_min_log_ - map_param_->unknown_flag_;
  //       map_data_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
  //       int idx                       = toAddress(x, y, z);
  //       map_data_->occupancy_buffer_[idx]    = map_param_->clamp_min_log_ - map_param_->unknown_flag_;
  //       map_data_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
  //   for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

  //     for (int y = min_cut_m(1); y < min_cut(1); ++y) {
  //       int idx                       = toAddress(x, y, z);
  //       map_data_->occupancy_buffer_[idx]    = map_param_->clamp_min_log_ - map_param_->unknown_flag_;
  //       map_data_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
  //       int idx                       = toAddress(x, y, z);
  //       map_data_->occupancy_buffer_[idx]    = map_param_->clamp_min_log_ - map_param_->unknown_flag_;
  //       map_data_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
  //   for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {

  //     for (int x = min_cut_m(0); x < min_cut(0); ++x) {
  //       int idx                       = toAddress(x, y, z);
  //       map_data_->occupancy_buffer_[idx]    = map_param_->clamp_min_log_ - map_param_->unknown_flag_;
  //       map_data_->distance_buffer_all_[idx] = 10000;
  //     }

  //     for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
  //       int idx                       = toAddress(x, y, z);
  //       map_data_->occupancy_buffer_[idx]    = map_param_->clamp_min_log_ - map_param_->unknown_flag_;
  //       map_data_->distance_buffer_all_[idx] = 10000;
  //     }
  //   }

  // update inflated occupied cells
  // clean outdated occupancy

  int inf_step = ceil(map_param_->obstacles_inflation_ / map_param_->resolution_);
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);

  for (int x = map_data_->local_bound_min_(0); x <= map_data_->local_bound_max_(0); ++x)
    for (int y = map_data_->local_bound_min_(1); y <= map_data_->local_bound_max_(1); ++y)
      for (int z = map_data_->local_bound_min_(2); z <= map_data_->local_bound_max_(2); ++z) {
        map_data_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate newest occpuied cells
  for (int x = map_data_->local_bound_min_(0); x <= map_data_->local_bound_max_(0); ++x)
    for (int y = map_data_->local_bound_min_(1); y <= map_data_->local_bound_max_(1); ++y)
      for (int z = map_data_->local_bound_min_(2); z <= map_data_->local_bound_max_(2); ++z) {
        int id1 = toAddress(x, y, z);
        if (map_data_->occupancy_buffer_[id1] > map_param_->min_occupancy_log_) {

          for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
            for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
              for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z) {
                Eigen::Vector3i inf_pt(x + inf_x, y + inf_y, z + inf_z);
                if (!isInMap(inf_pt)) continue;
                int inf_adr = toAddress(inf_pt);
                map_data_->occupancy_buffer_inflate_[inf_adr] = 1;
              }

          //   inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
          //   for (auto inf_pt : inf_pts) {
          //     if (!isInMap(inf_pt)) continue;
          //     int idx_inf = toAddress(inf_pt);
          //     if (idx_inf >= 0 &&
          //         idx_inf <
          //             map_param_->map_voxel_num_(0) * map_param_->map_voxel_num_(1) * map_param_->map_voxel_num_(2)) {
          //       map_data_->occupancy_buffer_inflate_[idx_inf] = 1;
          //     }
          //   }
        }
      }
}

double SDFMap::getResolution() {
  return map_param_->resolution_;
}

int SDFMap::getVoxelNum() {
  return map_param_->map_voxel_num_[0] * map_param_->map_voxel_num_[1] * map_param_->map_voxel_num_[2];
}

bool SDFMap::getBaseCoor(const int& id, Eigen::Vector4d& transform) {
  if (map_param_->no_drone_1_ && id == 1) {
    transform = Eigen::Vector4d(0, 0, 0, 0);
    return true;
  }

  auto iter = map_data_->swarm_transform_.find(id);
  if (iter == map_data_->swarm_transform_.end()) {
    ROS_ERROR("No transform of drone %d yet", id);
    return false;
  }
  transform = iter->second;
  return true;
}

void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
  ori = map_param_->map_origin_, size = map_param_->map_size_;
}

void SDFMap::getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax) {
  bmin = map_param_->box_mind_;
  bmax = map_param_->box_maxd_;
  // bmin = map_param_->vmin_;
  // bmax = map_param_->vmax_;
  // bmin[2] = map_param_->box_mind_[2];
  // bmax[2] = map_param_->box_maxd_[2];
}

void SDFMap::getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset) {
  bmin = map_data_->update_min_;
  bmax = map_data_->update_max_;
  if (reset) map_data_->reset_updated_box_ = true;
}

double SDFMap::getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) {
  if (!isInMap(pos)) {
    grad.setZero();
    return 0;
  }

  /* trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * map_param_->resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  posToIndex(pos_m, idx);
  Eigen::Vector3d idx_pos, diff;
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * map_param_->resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = getDistance(current_idx);
      }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * map_param_->resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * map_param_->resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= map_param_->resolution_inv_;

  return dist;
}
}  // namespace fast_planner
// SDFMap
