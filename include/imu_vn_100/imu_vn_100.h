/*
 * Copyright [2015] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IMU_VN_100_ROS_H_
#define IMU_VN_100_ROS_H_

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>

#include <vn100.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace imu_vn_100 {

namespace du = diagnostic_updater;
using TopicDiagnosticPtr = boost::shared_ptr<du::TopicDiagnostic>;

// NOTE: there is a DiagnosedPublisher inside diagnostic_updater
// but it does not have a default constructor thus we use this simple one
// instead, which has the same functionality
struct DiagnosedPublisher {
  ros::Publisher pub;
  TopicDiagnosticPtr diag;

  template <typename MessageT>
  void Create(ros::NodeHandle& pnh, const std::string& topic,
              du::Updater& updater, double& rate) {
    pub = pnh.advertise<MessageT>(topic, 1);
    du::FrequencyStatusParam freq_param(&rate, &rate, 0.01, 10);
    du::TimeStampStatusParam time_param(0, 0.5 / rate);
    diag = boost::make_shared<du::TopicDiagnostic>(topic, updater, freq_param,
                                                   time_param);
  }

  template <typename MessageT>
  void Publish(const MessageT& message) {
    diag->tick(message.header.stamp);
    pub.publish(message);
  }
};

/**
 * @brief Rotater contains the functions for rotating
 *        3 x 1 vectors and quaternions
 */

struct Rotater {
  // rotation from vectornav frame to body frame
  geometry_utils::Rot3 Rbv;
  geometry_utils::Quat qbv;
  geometry_utils::Quat q_nwu_ned;

  Rotater() {
    Rbv.eye();
    qbv = geometry_utils::RToQuat(Rbv);
    q_nwu_ned.x() = 1.0;
    q_nwu_ned.y() = 0.0;
    q_nwu_ned.z() = 0.0;
    q_nwu_ned.w() = 0.0;
  }

  ~Rotater() {return;}

  void setRotation(const geometry_utils::Rot3& rot) {
    // b = body reference frame
    // v = VectorNav VN-100 reference frame
    Rbv = rot;
    qbv = geometry_utils::RToQuat(Rbv);
  }

  geometry_utils::Vec3 sensorToBody(const geometry_utils::Vec3& in) const {
    return Rbv*in;
  }

  geometry_utils::Quat sensorToBody(const geometry_utils::Quat& in) const {
    /*********************************************************************
       Note: VectorNav gives an orientation corresponding to R_wned_v, the
       rotation that takes vectors from the VectorNav frame to the NED
       world frame. However, we want R_wnwu_b, the rotation that takes
       vectors from the desired body frame to the NWU world frame.

          R_wnwu_b = R_wnwu_wned * R_wned_v * R_b_v^T

     ********************************************************************/
    geometry_utils::Quat q_ned_v = in.normalize();
    return q_nwu_ned * q_ned_v * qbv.conj();
  }
};

/**
 * @brief ImuVn100 The class is a ros wrapper for the Imu class
 * @author Ke Sun
 */
class ImuVn100 {
 public:
  static constexpr int kBaseImuRate = 800;
  static constexpr int kDefaultImuRate = 100;
  static constexpr int kDefaultSyncOutRate = 20;

  explicit ImuVn100(const ros::NodeHandle& pnh);
  ImuVn100(const ImuVn100&) = delete;
  ImuVn100& operator=(const ImuVn100&) = delete;
  ~ImuVn100();

  void Initialize();

  void Stream(bool async = true);

  void PublishData(const VnDeviceCompositeData& data);

  void RequestOnce();

  void Idle(bool need_reply = true);

  void Resume(bool need_reply = true);

  void Disconnect();

  void Configure();

  struct SyncInfo {
    unsigned count = 0;
    ros::Time time;

    int rate = -1;
    double rate_double = -1;
    int pulse_width_us = 1000;
    int skip_count = 0;

    void Update(const unsigned sync_count, const ros::Time& sync_time);
    void FixSyncRate();
    bool SyncEnabled() const;
  };

  const SyncInfo sync_info() const { return sync_info_; }

  Rotater rotater_;

 private:
  ros::NodeHandle pnh_;
  Vn100 imu_;

  // Settings
  std::string port_;
  int baudrate_ = 921600;
  int imu_rate_ = kDefaultImuRate;
  double imu_rate_double_ = kDefaultImuRate;
  std::string frame_id_;

  bool enable_eulerzyx_ = true;
  bool enable_mag_ = true;
  bool enable_pres_ = true;
  bool enable_temp_ = true;
  bool binary_output_ = true;

  SyncInfo sync_info_;

  du::Updater updater_;
  DiagnosedPublisher pd_imu_, pd_mag_, pd_pres_, pd_temp_, pd_eulerzyx_;

  void FixImuRate();
  void LoadParameters();
  void CreateDiagnosedPublishers();
};

// Just don't like type that is ALL CAP
using VnErrorCode = VN_ERROR_CODE;
void VnEnsure(const VnErrorCode& error_code);

}  // namespace imu_vn_100

#endif  // IMU_VN_100_ROS_H_
