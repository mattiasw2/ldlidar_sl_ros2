/**
 * @file main.cpp
 * @author LDRobot (contact@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <algorithm>
#include <vector>

#include "ros2_api.h"
#include "ldlidar_driver.h"

// Host steady clock in nanoseconds. The driver stamps every point with it.
uint64_t GetTimestamp(void);

#define LIDAR_RANGE_MIN 0.1
#define LIDAR_RANGE_MAX 12.0

// Point stamps are on the steady clock; ROS time = steady time + this offset,
// re-measured once per revolution.
struct ScanClock {
  int64_t steady_to_ros_ns;
  rcl_clock_type_t clock_type;
  rclcpp::Time ToRos(uint64_t steady_ns) const {
    return rclcpp::Time(static_cast<int64_t>(steady_ns) + steady_to_ros_ns, clock_type);
  }
};

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, const ScanClock& clock, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

void  ToSensorPointCloudMessagePublish(ldlidar::Points2D& src, const ScanClock& clock, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& lidarpub);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // create a ROS2 Node
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); 

  std::string product_name;
	std::string laser_scan_topic_name;
  std::string point_cloud_2d_topic_name;
	std::string port_name;
  LaserScanSetting setting;
	setting.frame_id = "base_laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;
  setting.min_intensity = 0.0;
  int serial_baudrate = 0;
  bool do_filtering = true;
  setting.do_triplets = false;
  double nominal_speed_dps = 3600.0;  // LD14P / LD19P: 10 revolutions per second
  double speed_tolerance = 0.10;
  double settle_time_s = 1.0;
  
  ldlidar::LDType lidartypename = ldlidar::LDType::NO_VER;

  // Added to measure average beam count (points per revolution) at start:
  int beam_count = 0;
  int beam_count_i = 0;
  std::vector<int> beam_count_samples;

  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("laser_scan_topic_name", laser_scan_topic_name);
  node->declare_parameter<std::string>("point_cloud_2d_topic_name", point_cloud_2d_topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("serial_baudrate", serial_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);
  node->declare_parameter<int>("min_intensity", setting.min_intensity);
  node->declare_parameter<bool>("do_filtering", do_filtering);
  node->declare_parameter<bool>("do_triplets", setting.do_triplets);
  node->declare_parameter<double>("nominal_speed_dps", nominal_speed_dps);
  node->declare_parameter<double>("speed_tolerance", speed_tolerance);
  node->declare_parameter<double>("settle_time_s", settle_time_s);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("laser_scan_topic_name", laser_scan_topic_name);
  node->get_parameter("point_cloud_2d_topic_name", point_cloud_2d_topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("port_name", port_name);
  node->get_parameter("serial_baudrate", serial_baudrate);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);
  node->get_parameter("min_intensity", setting.min_intensity);
  node->get_parameter("do_filtering", do_filtering); // use Points2D Slbf::NearFilter
  node->get_parameter("do_triplets", setting.do_triplets); // combine beams into triplets, cut beams count by three
  node->get_parameter("nominal_speed_dps", nominal_speed_dps);
  node->get_parameter("speed_tolerance", speed_tolerance);
  node->get_parameter("settle_time_s", settle_time_s);

  ldlidar::LDLidarDriver* lidar_drv = new ldlidar::LDLidarDriver();

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is:%s", lidar_drv->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(node->get_logger(), "ROS2 param input:");
  RCLCPP_INFO(node->get_logger(), "<laser_scan_topic_name>: %s", laser_scan_topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<point_cloud_2d_topic_name>: %s", point_cloud_2d_topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<frame_id>: %s", setting.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_name>: %s ", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<serial_baudrate>: %d ", serial_baudrate);
  RCLCPP_INFO(node->get_logger(), "<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(node->get_logger(), "<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<angle_crop_min>: %f", setting.angle_crop_min);
  RCLCPP_INFO(node->get_logger(), "<angle_crop_max>: %f", setting.angle_crop_max);
  RCLCPP_INFO(node->get_logger(), "<min_intensity>: %d ", setting.min_intensity);
  RCLCPP_INFO(node->get_logger(), "<do_filtering>: %d ", do_filtering);
  RCLCPP_INFO(node->get_logger(), "<do_triplets>: %d ", setting.do_triplets);
  RCLCPP_INFO(node->get_logger(), "<nominal_speed_dps>: %.0f ", nominal_speed_dps);
  RCLCPP_INFO(node->get_logger(), "<speed_tolerance>: %.2f ", speed_tolerance);
  RCLCPP_INFO(node->get_logger(), "<settle_time_s>: %.2f ", settle_time_s);

  if (port_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), "fail, port_name is empty!");
    exit(EXIT_FAILURE);
  }

  lidar_drv->RegisterGetTimestampFunctional(std::bind(&GetTimestamp)); 
  lidar_drv->SetSpeedGate(nominal_speed_dps, speed_tolerance, settle_time_s);

  // make sure: .../ldlidar_driver/src/lipkg.cpp:224 - commented out exclusion of LD_14P_* in the if statement
  lidar_drv->EnableFilterAlgorithnmProcess(do_filtering);
  
  if(!strcmp(product_name.c_str(), "LDLiDAR_LD14")) {
    lidartypename = ldlidar::LDType::LD_14;
  } else if(!strcmp(product_name.c_str(), "LDLiDAR_LD14P")) {
    // Nominal measurement frequency, only used for plausibility limits. Point
    // timing is derived from each packet's angle span and speed instead, so
    // the LD19P's actual 5 kHz is handled correctly.
    lidartypename = ldlidar::LDType::LD_14P_4000HZ;
  } else {
    RCLCPP_ERROR(node->get_logger(),"Error, input param <product_name> is fail!!");
    exit(EXIT_FAILURE);
  }

  if (lidar_drv->Start(lidartypename, port_name, serial_baudrate)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar node start is success");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (lidar_drv->WaitLidarCommConnect(3500)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar communication is normal.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_laserscan = 
      node->create_publisher<sensor_msgs::msg::LaserScan>(laser_scan_topic_name, 10);
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_pointcloud = 
      node->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_2d_topic_name, 10);

  ldlidar::Points2D laser_scan_points_raw;
  ldlidar::Points2D laser_scan_points;

  uint64_t gate_dropped_reported = 0;
  rclcpp::Time gate_last_warn(0, 0, node->get_clock()->get_clock_type());

  RCLCPP_INFO(node->get_logger(), "start normal, pub lidar data");

  // No fixed publish rate: GetLaserScanData blocks until the driver has
  // assembled the next revolution, so every revolution is published once.
  while (rclcpp::ok() && ldlidar::LDLidarDriver::IsOk()) {

    // Speed gate report: at most one warning per 5 s, and only when the drop
    // count changed since the last report, so the final count after a stall
    // is always reported once.
    ldlidar::LiPkg::SpeedGateStats gate = lidar_drv->GetSpeedGateStats();
    uint64_t gate_dropped = gate.dropped_out_of_tolerance + gate.dropped_settling;
    if ((gate_dropped != gate_dropped_reported) &&
        ((node->now() - gate_last_warn).seconds() >= 5.0)) {
      gate_last_warn = node->now();
      RCLCPP_WARN(node->get_logger(),
        "speed gate: dropped %lu packets so far (%lu speed out of tolerance, %lu settling), "
        "last speed %.0f deg/s, nominal %.0f +-%.0f%%, %s",
        static_cast<unsigned long>(gate_dropped),
        static_cast<unsigned long>(gate.dropped_out_of_tolerance),
        static_cast<unsigned long>(gate.dropped_settling),
        gate.last_speed_dps, nominal_speed_dps, speed_tolerance * 100.0,
        gate.settled ? "accepting data again" : "waiting for the speed to settle");
      gate_dropped_reported = gate_dropped;
    }
  
    switch (lidar_drv->GetLaserScanData(laser_scan_points_raw, 1500)){
      case ldlidar::LidarStatus::NORMAL: {
        int n_points = static_cast<int>(laser_scan_points_raw.size());

        // Map the point stamps (steady clock) to ROS time, once per revolution.
        ScanClock scan_clock;
        {
          rclcpp::Time ros_now = node->now();
          uint64_t steady_now = GetTimestamp();
          scan_clock.steady_to_ros_ns = static_cast<int64_t>(ros_now.nanoseconds()) - static_cast<int64_t>(steady_now);
          scan_clock.clock_type = ros_now.get_clock_type();
        }

        int n_samples = 20;
        if(beam_count_i++ < n_samples) {
          // Measure average beam count (points per revolution) at start:
          if(beam_count_i > 1) {
            // skip the first sample, it is messed up
            beam_count_samples.push_back(n_points);
          }
          //RCLCPP_INFO(node->get_logger(), "LIDAR beam count: %d", n_points);
          if(beam_count_i == n_samples) {
            // Median, not mean: a single partial revolution in the samples
            // (e.g. after a system clock jump) would otherwise shift the
            // reference by more than the +-5 tolerance below and every later
            // revolution would be dropped.
            std::sort(beam_count_samples.begin(), beam_count_samples.end());
            beam_count = beam_count_samples[beam_count_samples.size() / 2];

            beam_count = (beam_count / 3) * 3; // ensure it consists of triples for filtering

            RCLCPP_INFO(node->get_logger(), "LIDAR: Median beam count: %d", beam_count);
          }
        } else if(abs(n_points - beam_count) < 5) {
          // ensure the size of points vector is constant between revolutions:
          laser_scan_points_raw.resize(beam_count, laser_scan_points_raw.back());

          if(setting.do_triplets) {

            laser_scan_points.clear();

            // combine beams into triplets, cut beams count by three:
            for(int i = 1; i < beam_count - 2 ;i += 3) {

              int p_intensity = 0;
              int p_distance = 0;
              int p_count = 0;

              for(int j=i-1; j < i+2 ;j++) {

                auto point =  laser_scan_points_raw.at(j);
                float range = point.distance / 1000.f;  // distance unit transform to meters
                int intensity = point.intensity;        // laser receive intensity 0...254

                if((intensity > setting.min_intensity) && (range > LIDAR_RANGE_MIN + 0.01) && (range < LIDAR_RANGE_MAX - 0.01)) {
                  // valid point - count and average:
                  p_distance += point.distance;
                  p_intensity += point.intensity;
                  p_count++;
                }
              }

              p_count = p_count == 0 ? 1 : p_count; // avoid division by zero when all three points hit infinity

              auto p = laser_scan_points_raw.at(i); // use mid-point as model
              auto pd = ldlidar::PointData(p.angle, p_distance / p_count, p_intensity / p_count, p.stamp, p.x, p.y);

              laser_scan_points.push_back(pd);
            }

            //RCLCPP_INFO(node->get_logger(), "LIDAR beam count: %d  %d  %d", beam_count, static_cast<int>(laser_scan_points_raw.size()), static_cast<int>(laser_scan_points.size()));

            ToLaserscanMessagePublish(laser_scan_points, scan_clock, setting, node, lidar_pub_laserscan);
            ToSensorPointCloudMessagePublish(laser_scan_points, scan_clock, setting, node, lidar_pub_pointcloud);

          } else {
 
            ToLaserscanMessagePublish(laser_scan_points_raw, scan_clock, setting, node, lidar_pub_laserscan);
            ToSensorPointCloudMessagePublish(laser_scan_points_raw, scan_clock, setting, node, lidar_pub_pointcloud);
 
          }
        } else {
          RCLCPP_WARN(node->get_logger(), "LIDAR beam count: %d (it should be around median %d)", n_points, beam_count);
        }
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        RCLCPP_ERROR(node->get_logger(), "ldlidar point cloud data publish time out, please check your lidar device.");
        lidar_drv->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT: {
        break;
      }
      default:
        break;
    }
  }

  lidar_drv->Stop();

  delete lidar_drv;
  lidar_drv = nullptr;

  RCLCPP_INFO(node->get_logger(), "this node of ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

uint64_t GetTimestamp(void) {
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch());
  return ((uint64_t)tmp.count());
}

// Earliest and latest point stamp of a revolution (steady clock, ns).
static void ScanTimeSpan(const ldlidar::Points2D& src, uint64_t& first_ns, uint64_t& last_ns) {
  first_ns = UINT64_MAX;
  last_ns = 0;
  for (const auto& point : src) {
    if (point.stamp < first_ns) first_ns = point.stamp;
    if (point.stamp > last_ns) last_ns = point.stamp;
  }
  if (first_ns == UINT64_MAX) {
    first_ns = 0;
  }
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, const ScanClock& clock, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  float angle_min, angle_max, angle_increment;

  int beam_size = static_cast<int>(src.size());
  if (beam_size <= 1) {
    return;
  }

  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);

  // Timing. Every point carries the time it was measured. The lidar rotates
  // clockwise, so with laser_scan_dir = true (counter-clockwise beam order,
  // the ROS convention) beam 0 is the LAST point measured and the beams run
  // backwards in time: header.stamp is the time of beam 0 and time_increment
  // is negative, so that beam i was measured at stamp + i * time_increment.
  // With laser_scan_dir = false beam 0 is the first point measured and
  // time_increment is positive. scan_time is the duration of the revolution.
  uint64_t first_ns, last_ns;
  ScanTimeSpan(src, first_ns, last_ns);
  double scan_time = static_cast<double>(last_ns - first_ns) / 1e9;
  double time_increment = scan_time / (double)(beam_size - 1);

  {
    sensor_msgs::msg::LaserScan output;
    if (setting.laser_scan_dir) {
      output.header.stamp = clock.ToRos(last_ns);
      output.time_increment = static_cast<float>(-time_increment);
    } else {
      output.header.stamp = clock.ToRos(first_ns);
      output.time_increment = static_cast<float>(time_increment);
    }
    output.scan_time = static_cast<float>(scan_time);
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = LIDAR_RANGE_MIN;
    output.range_max = LIDAR_RANGE_MAX;
    output.angle_increment = angle_increment;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;

      // Filter out unreasonable readings. Assign NaN to beam's variables to tell ROS there's nothing there:
      if ((point.distance == 0)
		      || (point.intensity < setting.min_intensity) // helps filtering out random noise
		      //|| (point.intensity > 250)
		      || (range < LIDAR_RANGE_MIN + 0.01) || (range > LIDAR_RANGE_MAX - 0.01)) {
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
      if (index < beam_size) {
        if (index < 0) {
          RCLCPP_ERROR(node->get_logger(), "error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f", 
            index, beam_size, angle, angle_min, angle_increment);
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index_anticlockwise])) {
            output.ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
            if (range < output.ranges[index_anticlockwise]) {
                output.ranges[index_anticlockwise] = range;
            }
          }
          output.intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index])) {
            output.ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                  //   value, it can be re assigned
            if (range < output.ranges[index]) {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = intensity;
        }
      }
    }
    lidarpub->publish(output);
  } 
}

void  ToSensorPointCloudMessagePublish(ldlidar::Points2D& src, const ScanClock& clock, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& lidarpub) {
  (void)node;

  // The cloud is stamped with the time of its earliest point.
  uint64_t first_ns, last_ns;
  ScanTimeSpan(src, first_ns, last_ns);
  rclcpp::Time start_scan_time = clock.ToRos(first_ns);

  ldlidar::Points2D dst = src;

  if (setting.laser_scan_dir) {
    for (auto&point : dst) {
      point.angle = 360.f - point.angle;
      if (point.angle < 0) {
        point.angle += 360.f;
      }
    }
  } 

  int frame_points_num = static_cast<int>(dst.size());

  sensor_msgs::msg::PointCloud2 output;

  output.header.stamp = start_scan_time;
  output.header.frame_id = setting.frame_id;

  // Unordered 2D cloud: one row, frame_points_num columns, fields x y z intensity (float32 each)
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(frame_points_num);
  output.height = 1;
  output.width = frame_points_num;
  output.is_dense = true;

  sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(output, "intensity");

  for (int i = 0; i < frame_points_num; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
    float range = dst[i].distance / 1000.f;  // distance unit transform to meters
    float intensity = dst[i].intensity;      // laser receive intensity 
    float dir_angle = ANGLE_TO_RADIAN(dst[i].angle);
    //  Convert polar coordinates to Cartesian coordinates:
    *iter_x = range * cos(dir_angle);
    *iter_y = range * sin(dir_angle);
    *iter_z = 0.0f;
    *iter_i = intensity;
  }
  lidarpub->publish(output);
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/
