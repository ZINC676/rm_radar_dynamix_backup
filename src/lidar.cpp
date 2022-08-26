//
// Created by jg on 2022/4/2.
//

#include "lidar.h"

namespace rm_radar
{
PLUGINLIB_EXPORT_CLASS(rm_radar::Lidar, nodelet::Nodelet)

Lidar::Lidar() : target_list_(TARGET_NUM, cv::Point3f(0, 0, 0))
{
  uv_queue_.reserve(PCL_QUEUE_SIZE);
}

Lidar::~Lidar()
{
  uv_queue_.clear();
  if (UI_thread.joinable())
    UI_thread.join();
}

void Lidar::onInit()
{
  nh_ = this->getMTPrivateNodeHandle();

  getParam();
//  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(camera_name_ + "output", 1);

  for (int i = 0; i < TARGET_NUM; i++)
  {
    ros::Publisher target_pub =
        nh_.advertise<std_msgs::Float32MultiArray>(camera_name_ + "_target_" + std::to_string(i + 1), 10);
    target_pub_list.emplace_back(target_pub);
  }

  ros::NodeHandle nh_reconfig(nh_, "lidar_reconfig");
  srv_ = new dynamic_reconfigure::Server<LidarConfig>(nh_reconfig);
  dynamic_reconfigure::Server<LidarConfig>::CallbackType cb = boost::bind(&Lidar::reconfigCB, this, _1, _2);
  srv_->setCallback(cb);

  pcl_sub_ = nh_.subscribe("/livox/lidar", 10, &Lidar::cloudCB, this);
  target_sub_1_ = nh_.subscribe<std_msgs::Float32MultiArray>("/" + camera_name_ + "/" + camera_name_ + "_roi_data1", 5,
                                                             &Lidar::targetCB<1>, this);
  target_sub_2_ = nh_.subscribe<std_msgs::Float32MultiArray>("/" + camera_name_ + "/" + camera_name_ + "_roi_data2", 5,
                                                             &Lidar::targetCB<2>, this);
  target_sub_3_ = nh_.subscribe<std_msgs::Float32MultiArray>("/" + camera_name_ + "/" + camera_name_ + "_roi_data3", 5,
                                                             &Lidar::targetCB<3>, this);
  target_sub_4_ = nh_.subscribe<std_msgs::Float32MultiArray>("/" + camera_name_ + "/" + camera_name_ + "_roi_data4", 5,
                                                             &Lidar::targetCB<4>, this);
  target_sub_5_ = nh_.subscribe<std_msgs::Float32MultiArray>("/" + camera_name_ + "/" + camera_name_ + "_roi_data5", 5,
                                                             &Lidar::targetCB<5>, this);

}

void Lidar::getParam()
{
  assert(nh_.getParam("camera_name", camera_name_));

  int intrinsic_rows = 0;
  int intrinsic_cols = 0;
  nh_.getParam(camera_name_ + "_config/camera_matrix/rows", intrinsic_rows);
  nh_.getParam(camera_name_ + "_config/camera_matrix/cols", intrinsic_cols);
  std::vector<float> intrinsic;
  assert(nh_.getParam(camera_name_ + "_config/camera_matrix/data", intrinsic));
  if (intrinsic.size() != 3 * 3)
    exit(1);
  intrinsic_ = cv::Matx<float, 3, 3>(intrinsic[0], intrinsic[1], intrinsic[2], intrinsic[3], intrinsic[4], intrinsic[5],
                                     intrinsic[6], intrinsic[7], intrinsic[8]);

  int extrinsic_rows = 0;
  int extrinsic_cols = 0;
  nh_.getParam(camera_name_ + "_config/extrinsic/rows", extrinsic_rows);
  nh_.getParam(camera_name_ + "_config/extrinsic/cols", extrinsic_cols);
  std::vector<float> extrinsic(extrinsic_rows * extrinsic_cols);
  nh_.getParam(camera_name_ + "_config/extrinsic/data", extrinsic);
  if (extrinsic.size() != 3 * 4)
    exit(1);
  extrinsic_ =
      cv::Matx<float, 3, 4>(extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3], extrinsic[4], extrinsic[5],
                            extrinsic[6], extrinsic[7], extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]);

  in_dot_ex_ = intrinsic_ * extrinsic_;

  int distortion_rows = 0;
  int distortion_cols = 0;
  nh_.getParam(camera_name_ + "_config/distortion_coefficients/rows", distortion_rows);
  nh_.getParam(camera_name_ + "_config/distortion_coefficients/cols", distortion_cols);
  std::vector<float> distortion;
  nh_.getParam(camera_name_ + "_config/distortion_coefficients/data", distortion);
  if (distortion.size() != 1 * 5)
    exit(1);
  distortion_ = cv::Matx<float, 1, 5>(distortion[0], distortion[1], distortion[2], distortion[3], distortion[4]);

  int projection_rows = 0;
  int projection_cols = 0;
  nh_.getParam(camera_name_ + "_config/projection_matrix/rows", projection_rows);
  nh_.getParam(camera_name_ + "_config/projection_matrix/cols", projection_cols);
  std::vector<float> projection(extrinsic_rows * projection_cols);
  nh_.getParam(camera_name_ + "_config/projection_matrix/data", projection);
  if (projection.size() != 3 * 4)
    exit(1);
  projection_ =
      cv::Matx<float, 3, 4>(projection[0], projection[1], projection[2], projection[3], projection[4], projection[5],
                            projection[6], projection[7], projection[8], projection[9], projection[10], projection[11]);

  int world_to_cam_rows = 0;
  int world_to_cam_cols = 0;
  nh_.getParam(camera_name_ + "_config/world_to_cam/rows", world_to_cam_rows);
  nh_.getParam(camera_name_ + "_config/world_to_cam/cols", world_to_cam_cols);
  std::vector<float> world_to_cam(world_to_cam_rows * world_to_cam_cols);
  nh_.getParam(camera_name_ + "_config/world_to_cam/data", world_to_cam);
  if (world_to_cam.size() != 3 * 4)
    exit(1);
  cam_to_world_ =
      cv::Matx<float, 4, 4>(world_to_cam[0], world_to_cam[1], world_to_cam[2], world_to_cam[3], world_to_cam[4], world_to_cam[5],
                            world_to_cam[6], world_to_cam[7], world_to_cam[8], world_to_cam[9], world_to_cam[10], world_to_cam[11],0,0,0,1).inv();
}

void Lidar::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud_xyz);

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//  filter_.setInputCloud(cloud_xyz);
//  filter_.setRadiusSearch(radius_search_);
//  filter_.setMinNeighborsInRadius(min_neighbors_in_radius_);
//  filter_.filter(*cloud_filtered);

  boost::shared_ptr<std::vector<PointUV>> uv_with_depth(new std::vector<PointUV>);
  getUVDepth(cloud_xyz, uv_with_depth);

  {
    boost::unique_lock<boost::shared_mutex> write_mutex(pcl_queue_lock_);
    if (uv_queue_.size() >= PCL_QUEUE_SIZE)
      uv_queue_.erase(uv_queue_.cbegin());
    uv_queue_.emplace_back(uv_with_depth);
  }

  //    pcl::toROSMsg(*cloud_filtered, output_);
  //    output_.header.frame_id = camera_name_+"pcl";
  //    pcl_pub_.publish(output_);
}

void Lidar::reconfigCB(LidarConfig& config, uint32_t level)
{
  (void)level;

  radius_search_ = config.radius_search;
  min_neighbors_in_radius_ = config.min_neighbors_in_radius;

  rot_x_ = cv::Matx<float, 4, 4>(1, 0, 0, 0, 0, cos(config.rot_x * PI / 180), -sin(config.rot_x * PI / 180), 0, 0,
                                 sin(config.rot_x * PI / 180), cos(config.rot_x * PI / 180), 0, 0, 0, 0, 1);
  rot_y_ = cv::Matx<float, 4, 4>(cos(config.rot_y * PI / 180), 0, sin(config.rot_y * PI / 180), 0, 0, 1, 0, 0,
                                 -sin(config.rot_y * PI / 180), 0, cos(config.rot_y * PI / 180), 0, 0, 0, 0, 1);
  rot_z_ =
      cv::Matx<float, 4, 4>(cos(config.rot_z * PI / 180), -sin(config.rot_z * PI / 180), 0, 0,
                            sin(config.rot_z * PI / 180), cos(config.rot_z * PI / 180), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  trans_ = cv::Matx<float, 4, 4>(1, 0, 0, config.trans_x, 0, 1, 0, config.trans_y, 0, 0, 1, config.trans_z, 0, 0, 0, 1);
  extrinsic_1_ = trans_ * rot_x_ * rot_y_ * rot_z_;
  is_extrinsic_ = config.extrinsic;
}

template <const int ROBOT_ID>
void Lidar::targetCB(const std_msgs::Float32MultiArrayConstPtr& roi_msg)
{
  cv::Matx22f roi_points =
      cv::Matx<float, 2, 2>(roi_msg->data[0], roi_msg->data[1], roi_msg->data[2], roi_msg->data[3]);
  cv::Point3f target(0, 0, 0);
  if (getTarget(&roi_points, target))
  {
    std_msgs::Float32MultiArray target_msg;
    target_msg.data = { target.x, target.y, target.z };
    target_list_[ROBOT_ID - 1] = target;

    ROS_INFO(("target_" + std::to_string(ROBOT_ID) + "(%f,%f,%f)").c_str(), target.x, target.y, target.z);

    target_pub_list[ROBOT_ID - 1].publish(target_msg);
//    target_broadcaster_.sendTransform(
//        tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(target.x, target.y, target.z)),
//                             ros::Time::now(), "livox_frame", "target_" + std::to_string(ROBOT_ID)));
  }
}

bool Lidar::getTarget(cv::Matx22f* roi_points, cv::Point3f& target)
{
  if (uv_queue_.size() >= PCL_QUEUE_SIZE)
  {
    if (roi_points->dot(*roi_points) != 0)
    {
      std::vector<cv::Point2f> roi_points_vec;
      cv::Point2f roi_tl(roi_points->val[0], roi_points->val[1]);
      cv::Point2f roi_br(roi_points->val[2], roi_points->val[3]);
      roi_points_vec.emplace_back(roi_tl);
      roi_points_vec.emplace_back(roi_br);

      cv::undistortPoints(roi_points_vec, roi_points_vec, intrinsic_, distortion_, cv::noArray(), projection_);

      double depth = 0;
      int sum = 0;
      cv::Point2f tl(std::min(roi_points_vec[0].x, roi_points_vec[1].x),
                     std::min(roi_points_vec[0].y, roi_points_vec[1].y));
      cv::Point2f br(std::max(roi_points_vec[0].x, roi_points_vec[1].x),
                     std::max(roi_points_vec[0].y, roi_points_vec[1].y));

      {
        boost::shared_lock<boost::shared_mutex> read_mutex(pcl_queue_lock_);
        for (auto& i : uv_queue_)  // todo kdtree
        {
          for (auto& j : *i)
          {
            if (j.u > tl.x && j.u < br.x)
              if (j.v > tl.y && j.v < br.y)
              {
                depth = depth + j.depth;
                sum++;
              }
          }
        }
      }

      if (sum)
      {
        PointUV target_depth;
        target_depth.u = (tl.x + br.x) / 2;
        target_depth.v = (tl.y + br.y) / 2;
        target_depth.depth = depth / sum;

        target = coordinateUVToCam(target_depth);

        cv::Matx41f target_in_world=cam_to_world_*cv::Matx41f(target.x,target.y,target.z,1);
        target=cv::Point3f(target_in_world.val[0],target_in_world.val[1],target_in_world.val[2]);
        std::cout<<target<<std::endl;

        if (target.x > SITE_WIDTH || target.y > SITE_HIGH || target.x < 0 || target.y < 0)
          return false;
        return true;
      }
      else
        std::cout << "sum=" << sum << std::endl;
    }
  }

  return false;
}

cv::Point3f Lidar::coordinateUVToCam(const PointUV& uv_depth)
{
  cv::Point3f target_in_cam;
  target_in_cam.x = (uv_depth.u - projection_(0, 2) - projection_(0, 3)) / projection_(0, 0) * uv_depth.depth;
  target_in_cam.y = (uv_depth.v - projection_(1, 2) - projection_(1, 3)) / projection_(1, 1) * uv_depth.depth;
  target_in_cam.z = 1.0 * uv_depth.depth;
  return target_in_cam;
}

cv::Point3f Lidar::coordinateCamToWorld(const cv::Point3f& target_in_cam)
{
  cv::Point3f target_in_world;
  cv::Matx41f xyz_w, xyz_c;
  xyz_c = cv::Matx<float, 4, 1>(target_in_cam.x, target_in_cam.y, target_in_cam.z, 1);
  cv::Matx44f extrinsic;
  if (is_extrinsic_)
  {
    extrinsic = extrinsic_1_;
  }
  else
  {
    extrinsic =
        cv::Matx<float, 4, 4>(extrinsic_.val[0], extrinsic_.val[1], extrinsic_.val[2], extrinsic_.val[3],
                              extrinsic_.val[4], extrinsic_.val[5], extrinsic_.val[6], extrinsic_.val[7],
                              extrinsic_.val[8], extrinsic_.val[9], extrinsic_.val[10], extrinsic_.val[11], 0, 0, 0, 1);
  }
  xyz_w = extrinsic.inv() * xyz_c;
  target_in_world.x = xyz_w(0, 0);
  target_in_world.y = xyz_w(1, 0);
  target_in_world.z = xyz_w(2, 0);
  return target_in_world;
}

void Lidar::getUVDepth(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       boost::shared_ptr<std::vector<PointUV>>& uv_with_depth)
{
  for (auto& i : *cloud)
  {
    cv::Matx31f uv_depth = in_dot_ex_ * cv::Matx<float, 4, 1>(i.x, i.y, i.z, 1);
    PointUV point;
    point.u = uv_depth.val[0] / uv_depth.val[2];
    point.v = uv_depth.val[1] / uv_depth.val[2];
    point.depth = uv_depth.val[2];
    uv_with_depth->emplace_back(point);
  }
}
}  // namespace rm_radar
