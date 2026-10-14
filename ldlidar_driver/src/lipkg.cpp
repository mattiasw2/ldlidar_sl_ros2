/**
 * @file lipkg.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
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
#include "lipkg.h"

#include <math.h>

#include <algorithm>

namespace ldlidar {

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LiPkg::LiPkg()
  : lidar_measure_freq_(2300),
    typenumber_(LDType::NO_VER),
    lidarstatus_(LidarStatus::NORMAL),
    lidarerrorcode_(LIDAR_NO_ERROR),
    is_frame_ready_(false),
    is_noise_filter_(false),
    timestamp_(0),
    speed_(0),
    get_timestamp_(nullptr),
    is_poweron_comm_normal_(false),
    poweron_datapkg_count_(0),
    last_pkg_timestamp_(0),
    have_last_raw_ts_(false),
    last_raw_ts_(0),
    lidar_ms_base_(0),
    gate_nominal_dps_(3600.0),
    gate_tolerance_(0.10),
    gate_settle_ns_(1000000000ULL),
    gate_ok_since_ns_(0),
    gate_settled_(false),
    gate_dropped_out_of_tolerance_(0),
    gate_dropped_settling_(0),
    gate_settled_flag_(false),
    last_packet_time_ns_(0) {

}

LiPkg::~LiPkg() {

}

void LiPkg::SetProductType(LDType typenumber) {
  typenumber_ = typenumber;
  switch (typenumber) {
    case LDType::LD_14:
    case LDType::LD_14P_2300HZ:
      lidar_measure_freq_ = 2300;
      break;
    case LDType::LD_14P_4000HZ:
      lidar_measure_freq_ = 4000;
      break;
    default :
      lidar_measure_freq_ = 2300;
      break;
  }
}

void LiPkg::SetNoiseFilter(bool is_enable) {
  is_noise_filter_ = is_enable;
}

void LiPkg::RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle) {
  get_timestamp_ = timestamp_handle;
}

void LiPkg::SetSpeedGate(double nominal_dps, double tolerance, double settle_s) {
  gate_nominal_dps_ = nominal_dps;
  gate_tolerance_ = tolerance;
  gate_settle_ns_ = (settle_s > 0) ? static_cast<uint64_t>(settle_s * 1e9) : 0;
  gate_ok_since_ns_ = 0;
  gate_settled_ = false;
  gate_settled_flag_ = false;
}

LiPkg::SpeedGateStats LiPkg::GetSpeedGateStats(void) {
  SpeedGateStats st;
  st.dropped_out_of_tolerance = gate_dropped_out_of_tolerance_.load();
  st.dropped_settling = gate_dropped_settling_.load();
  st.last_speed_dps = speed_.load();
  st.settled = gate_settled_flag_.load();
  return st;
}

uint64_t LiPkg::GetLastPacketTime(void) {
  return last_packet_time_ns_.load();
}

uint64_t LiPkg::UnwrapLidarTimestamp(uint16_t raw_ms) {
  if (have_last_raw_ts_ && (raw_ms < last_raw_ts_)) {
    // The vendor manual says the counter resets when it reaches 30000 ms.
    // Should a unit ever count beyond that, treat it as a plain 16 bit wrap.
    lidar_ms_base_ += (last_raw_ts_ < kLidarTimestampWrapMs) ? kLidarTimestampWrapMs : 65536;
  }
  have_last_raw_ts_ = true;
  last_raw_ts_ = raw_ms;
  return (lidar_ms_base_ + raw_ms) * 1000000ULL;
}

int64_t LiPkg::UpdateClockOffset(uint64_t lidar_ns, uint64_t host_rx_ns) {
  // offset = host receive time - lidar time. USB and scheduling latency only
  // ever make it larger, so the minimum over a sliding window is the best
  // estimate of the true offset. Kept as a monotonic deque: offsets increase
  // from front to back, so the front is the window minimum.
  int64_t offset = static_cast<int64_t>(host_rx_ns) - static_cast<int64_t>(lidar_ns);
  while (!offset_window_.empty() && (offset_window_.back().offset_ns >= offset)) {
    offset_window_.pop_back();
  }
  offset_window_.push_back(OffsetSample{lidar_ns, offset});
  while (!offset_window_.empty() &&
         (offset_window_.front().lidar_ns + kOffsetWindowNs < lidar_ns)) {
    offset_window_.pop_front();
  }
  return offset_window_.front().offset_ns;
}

bool LiPkg::SpeedGateAccepts(double speed_dps, uint64_t host_rx_ns) {
  bool in_tolerance = fabs(speed_dps - gate_nominal_dps_) <= (gate_tolerance_ * gate_nominal_dps_);
  if (!in_tolerance) {
    gate_dropped_out_of_tolerance_++;
    gate_ok_since_ns_ = 0;
    gate_settled_ = false;
    gate_settled_flag_ = false;
    return false;
  }
  if (gate_settled_) {
    return true;
  }
  if (0 == gate_ok_since_ns_) {
    gate_ok_since_ns_ = host_rx_ns;
  }
  if ((host_rx_ns - gate_ok_since_ns_) < gate_settle_ns_) {
    gate_dropped_settling_++;
    return false;
  }
  gate_settled_ = true;
  gate_settled_flag_ = true;
  return true;
}

bool LiPkg::AnalysisOne(uint8_t byte) {
  static enum {
    HEADER,
    VER_LEN,
    DATA,
  } state = HEADER;
  static uint16_t count = 0;
  static uint8_t tmp[128] = {0};
  static uint16_t pkg_count = sizeof(LiDARFrameTypeDef);

  switch (state) {
    case HEADER:
      if (byte == PKG_HEADER) {
        tmp[count++] = byte;
        state = VER_LEN;
      }
      break;
    case VER_LEN:
      if (byte == PKG_VER_LEN) {
        tmp[count++] = byte;
        state = DATA;
      } else {
        state = HEADER;
        count = 0;
        return false;
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        memcpy((uint8_t *)&datapkg_, tmp, pkg_count);
        uint8_t crc = CalCRC8((uint8_t *)&datapkg_, pkg_count - 1);
        state = HEADER;
        count = 0;
        if (crc == datapkg_.crc8) {
          return true;
        } else {
          return false;
        }
      }
      break;
    default:
      break;
  }

  return false;  
}

bool LiPkg::Parse(const uint8_t *data, long len) {
  for (int i = 0; i < len; i++) {
    if (AnalysisOne(data[i])) {
      uint64_t host_rx_ns = get_timestamp_();
      last_packet_time_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();

      poweron_datapkg_count_++;
      if (poweron_datapkg_count_ >= 2) {
        poweron_datapkg_count_ = 0;
        is_poweron_comm_normal_ = true;
      }
      
      speed_ = datapkg_.speed;
      timestamp_ = datapkg_.timestamp;

      // Keep the lidar clock mapping running on every packet so that the
      // offset estimate is ready as soon as the speed gate opens.
      uint64_t lidar_ns = UnwrapLidarTimestamp(datapkg_.timestamp);
      int64_t offset_ns = UpdateClockOffset(lidar_ns, host_rx_ns);

      if (!SpeedGateAccepts(datapkg_.speed, host_rx_ns)) {
        // Bad speed means the angles inside the partial revolution are not
        // trustworthy either: start the revolution over.
        frame_tmp_.clear();
        continue;
      }
      
      // parse a package is success
      double diff = (datapkg_.end_angle / 100 - datapkg_.start_angle / 100 + 360) % 360;
      if (diff <= ((double)datapkg_.speed * (double)POINT_PER_PACK / lidar_measure_freq_ * 1.5)) {
        // Packet time on the host clock. The vendor manual only says the
        // field is "the timestamp of the data packet"; it is taken to mark
        // the last measurement, the moment the packet is complete.
        uint64_t pkg_host_ns = static_cast<uint64_t>(static_cast<int64_t>(lidar_ns) + offset_ns);

        uint32_t span = ((uint32_t)datapkg_.end_angle + 36000 - (uint32_t)datapkg_.start_angle) % 36000;
        // Time between the 12 measurements: the packet covers span degrees at
        // speed deg/s. This follows the real measurement rate (5 kHz on the
        // LD19P, 4 kHz on the LD14P) without having to know it.
        double pkg_duration_s;
        if (datapkg_.speed > 0) {
          pkg_duration_s = (span / 100.0) / (double)datapkg_.speed;
        } else {
          pkg_duration_s = (double)(POINT_PER_PACK - 1) / (double)lidar_measure_freq_;
        }
        double point_step_ns = pkg_duration_s * 1e9 / (double)(POINT_PER_PACK - 1);

        float step = span / (POINT_PER_PACK - 1) / 100.0;
        float start = (double)datapkg_.start_angle / 100.0;
        PointData data;
        for (int i = 0; i < POINT_PER_PACK; i++) {
          data.distance = datapkg_.point[i].distance;
          data.angle = start + i * step;
          if (data.angle >= 360.0) {
            data.angle -= 360.0;
          }
          data.intensity = datapkg_.point[i].intensity;
          data.stamp = pkg_host_ns - static_cast<uint64_t>(point_step_ns * (POINT_PER_PACK - 1 - i));
          frame_tmp_.push_back(PointData(data.angle, data.distance, data.intensity, data.stamp));
        }
        last_pkg_timestamp_ = pkg_host_ns;
      }
    }
  }

  return true;
}

bool LiPkg::AssemblePacket() {
  float last_angle = 0;
  Points2D tmp, data;
  int count = 0;

  if (speed_ <= 0) {
    frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.end());
    return false;
  }

  for (auto n : frame_tmp_) {
    // wait for enough data, need enough data to show a circle
	// enough data has been obtained
    if ((n.angle < 20.0) && (last_angle > 340.0)) {
      if ((count * GetSpeed()) > (lidar_measure_freq_ * 1.4)) {
        if (count >= (int)frame_tmp_.size()) {
          frame_tmp_.clear();
        } else {
          frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
        }
        return false;
      }
      data.insert(data.begin(), frame_tmp_.begin(), frame_tmp_.begin() + count);

      SlTransform trans(typenumber_);
      data = trans.Transform(data); // transform raw data to stantard data  
    
      if (is_noise_filter_) {
        // We need filtering for LD19P (LD14P) too, so - commented out the following:
        // && (typenumber_ != ldlidar::LDType::LD_14P_2300HZ) && (typenumber_ != ldlidar::LDType::LD_14P_4000HZ)) {
        Slbf sb(speed_);
        tmp = sb.NearFilter(data); // filter noise point
      } else {
        tmp = data;
      }
      
      std::sort(tmp.begin(), tmp.end(), [](PointData a, PointData b) { return a.stamp < b.stamp; });
      if (tmp.size() > 0) {
        SetLaserScanData(tmp);
        SetFrameReady();

        if (count >= (int)frame_tmp_.size()) {
          frame_tmp_.clear();
        } else {
          frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
        }
        return true;
      }
    }
    count++;

    if ((count * GetSpeed()) > (lidar_measure_freq_ * 2)) {
      if (count >= (int)frame_tmp_.size()) {
        frame_tmp_.clear();
      } else {
        frame_tmp_.erase(frame_tmp_.begin(), frame_tmp_.begin() + count);
      }
      return false;
    }

    last_angle = n.angle;
  }

  return false;
}

void LiPkg::CommReadCallBack(const char *byte, size_t len) {
  if (this->Parse((uint8_t *)byte, len)) {
    this->AssemblePacket();
  }
}

bool LiPkg::GetLaserScanData(Points2D& out) {
  if (IsFrameReady()) {
    ResetFrameReady();
    out = GetLaserScanData();
    return true;
  } else {
    return false;
  }
}

bool LiPkg::WaitLaserScanData(Points2D& out, int timeout_ms) {
  {
    std::unique_lock<std::mutex> lk(mutex_lock1_);
    if (!frame_ready_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                  [this] { return is_frame_ready_; })) {
      return false;
    }
    is_frame_ready_ = false;
  }
  out = GetLaserScanData();
  return true;
}

double LiPkg::GetSpeed(void) { 
  return (speed_ / 360.0);  // unit  is Hz
}

LidarStatus LiPkg::GetLidarStatus(void) {
  return lidarstatus_;
}

uint8_t LiPkg::GetLidarErrorCode(void) {
  return lidarerrorcode_;
}

bool LiPkg::GetLidarPowerOnCommStatus(void) {
  if (is_poweron_comm_normal_) {
    is_poweron_comm_normal_ = false;
    return true;
  } else {
    return false;
  }
}

void LiPkg::SetLidarStatus(LidarStatus status) {
  lidarstatus_ = status;
}

void LiPkg::SetLidarErrorCode(uint8_t errorcode) {
  lidarerrorcode_ = errorcode;
}

bool LiPkg::IsFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  return is_frame_ready_; 
}

void LiPkg::ResetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = false;
}

void LiPkg::SetFrameReady(void) {
  {
    std::lock_guard<std::mutex> lg(mutex_lock1_);
    is_frame_ready_ = true;
  }
  frame_ready_cv_.notify_all();
}

void LiPkg::SetLaserScanData(Points2D& src) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  lidar_frame_data_ = src;
}

Points2D LiPkg::GetLaserScanData(void) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  return lidar_frame_data_; 
}

}  // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/