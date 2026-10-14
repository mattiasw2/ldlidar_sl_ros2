/**
 * @file lipkg.h
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
#ifndef __LIPKG_H
#define __LIPKG_H

#include <string.h>

#include <stdint.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>

#include "slbf.h"
#include "sl_transform.h"

namespace ldlidar {

enum {
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t speed;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

class LiPkg {
public:
  LiPkg();

  ~LiPkg();

  void SetProductType(LDType typenumber);

  void SetNoiseFilter(bool is_enable);

  /**
   * @brief register the host clock used for point stamps. It must be a
   *        monotonic (steady) clock in nanoseconds.
  */
  void RegisterTimestampGetFunctional(std::function<uint64_t(void)> timestamp_handle);

  /**
   * @brief motor speed gate. A packet is dropped when
   *        |speed - nominal_dps| > tolerance * nominal_dps, and after start-up
   *        or any dropped packet, data is only accepted again once the speed
   *        has been within tolerance for settle_s seconds.
   *        tolerance >= 1.0 disables the gate.
  */
  void SetSpeedGate(double nominal_dps, double tolerance, double settle_s);

  struct SpeedGateStats {
    uint64_t dropped_out_of_tolerance;  // packets with speed outside the tolerance band
    uint64_t dropped_settling;          // in-tolerance packets dropped while waiting for settle_s
    double last_speed_dps;              // speed field of the last CRC-valid packet
    bool settled;                       // true when data is currently being accepted
  };

  SpeedGateStats GetSpeedGateStats(void);

  void CommReadCallBack(const char *byte, size_t len);

  /**
   * @brief get lidar scan data (non-blocking)
  */
  bool GetLaserScanData(Points2D& out); 

  /**
   * @brief wait up to timeout_ms for the next complete revolution.
   * @retval true when a revolution was copied into out.
  */
  bool WaitLaserScanData(Points2D& out, int timeout_ms);

  /**
   * @brief steady-clock time (ns) of the last CRC-valid packet, 0 if none yet.
  */
  uint64_t GetLastPacketTime(void);

  /**
   * @brief get Lidar spin speed (Hz)
  */
  double GetSpeed(void);

  LidarStatus GetLidarStatus(void);

  uint8_t GetLidarErrorCode(void);

  bool GetLidarPowerOnCommStatus(void);

  void ClearDataProcessStatus(void) {
    is_frame_ready_ = false;
    is_poweron_comm_normal_ = false;
    lidarstatus_ = LidarStatus::NORMAL;
    lidarerrorcode_ = LIDAR_NO_ERROR;
    last_pkg_timestamp_ = 0;
    have_last_raw_ts_ = false;
    last_raw_ts_ = 0;
    lidar_ms_base_ = 0;
    offset_window_.clear();
    gate_ok_since_ns_ = 0;
    gate_settled_ = false;
    last_packet_time_ns_ = 0;
    frame_tmp_.clear();
  }

private:
  int lidar_measure_freq_;
  LDType typenumber_;
  LidarStatus lidarstatus_;
  uint8_t lidarerrorcode_;
  bool is_frame_ready_;
  bool is_noise_filter_;
  uint16_t timestamp_; 
  std::atomic<double> speed_;
  std::function<uint64_t(void)> get_timestamp_;
  bool is_poweron_comm_normal_;
  uint8_t poweron_datapkg_count_;
  uint64_t last_pkg_timestamp_;

  // lidar clock (packet timestamp field, ms, wraps at 30000) unwrapped into a
  // monotonic value, and its offset to the host steady clock
  bool have_last_raw_ts_;
  uint16_t last_raw_ts_;
  uint64_t lidar_ms_base_;
  struct OffsetSample {
    uint64_t lidar_ns;
    int64_t offset_ns;  // host_rx_ns - lidar_ns
  };
  std::deque<OffsetSample> offset_window_;  // increasing offset_ns, front is the minimum
  static const uint64_t kOffsetWindowNs = 2000000000ULL;  // 2 s
  static const uint32_t kLidarTimestampWrapMs = 30000;

  // speed gate
  double gate_nominal_dps_;
  double gate_tolerance_;
  uint64_t gate_settle_ns_;
  uint64_t gate_ok_since_ns_;  // host time of the first in-tolerance packet of the current run, 0 = none
  bool gate_settled_;
  std::atomic<uint64_t> gate_dropped_out_of_tolerance_;
  std::atomic<uint64_t> gate_dropped_settling_;
  std::atomic<bool> gate_settled_flag_;
  std::atomic<uint64_t> last_packet_time_ns_;

  LiDARFrameTypeDef datapkg_;
  Points2D lidar_frame_data_;
  Points2D frame_tmp_;
  std::mutex mutex_lock1_;
  std::mutex mutex_lock2_;
  std::condition_variable frame_ready_cv_;

  uint64_t UnwrapLidarTimestamp(uint16_t raw_ms);  // returns monotonic lidar time in ns

  int64_t UpdateClockOffset(uint64_t lidar_ns, uint64_t host_rx_ns);  // returns the current min offset

  bool SpeedGateAccepts(double speed_dps, uint64_t host_rx_ns);

  void SetLidarStatus(LidarStatus status);

  void SetLidarErrorCode(uint8_t errorcode);

  bool AnalysisOne(uint8_t byte); // parse single packet

  bool Parse(const uint8_t *data, long len); 

  bool AssemblePacket(); // combine stantard data into data frames and calibrate

  bool IsFrameReady(void);  // get Lidar data frame ready flag

  void ResetFrameReady(void);  // reset frame ready flag

  void SetFrameReady(void);    // set frame ready flag

  void SetLaserScanData(Points2D& src);

  Points2D GetLaserScanData(void);

  // void AnalysisLidarIsBlocking(uint16_t lidar_speed_val);

  // void AnalysisLidarIsOcclusion(Points2D& lidar_data);
};

} // namespace ldlidar

#endif  // __LIPKG_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/