#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <tf2/LinearMath/Transform.h>

#include <gps_tools/conversions.h>
#include <ublox_gps/dataspeed_product.hpp>
#include <ublox_gps/utils.hpp>

namespace ublox_node {

DataspeedProduct::DataspeedProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node)
  : HpgRefProduct(nav_rate, meas_rate, updater, rtcms, node)
{
  if (getRosBoolean(node_, "publish.ds.odom") || getRosBoolean(node_, "publish.ds.all")) {
    odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  }
  if (getRosBoolean(node_, "publish.ds.imu") || getRosBoolean(node_, "publish.ds.all")) {
    imu_pub = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    ds_imu_pub = node_->create_publisher<ublox_msgs::msg::DsIMU>("dsimu", 1);
  }
  if (getRosBoolean(node_, "publish.ds.hp_fix") || getRosBoolean(node_, "publish.ds.all")) {
    fix_pub = node_->create_publisher<sensor_msgs::msg::NavSatFix>("hp_fix", 1);
  }
}

void DataspeedProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  if (getRosBoolean(node_, "publish.ds.imu") || getRosBoolean(node_, "publish.ds.odom") || getRosBoolean(node_, "publish.ds.all")) {
    gps->subscribe<ublox_msgs::msg::DsIMU>(std::bind(&DataspeedProduct::callbackDsImu, this, std::placeholders::_1), 1);
  }
  if (getRosBoolean(node_, "publish.ds.hp_fix") || getRosBoolean(node_, "publish.ds.odom") || getRosBoolean(node_, "publish.ds.all")) {
    gps->subscribe<ublox_msgs::msg::NavHPPOSLLH>(std::bind(&DataspeedProduct::callbackNavHpPosLlh, this, std::placeholders::_1), 1);
    gps->subscribe<ublox_msgs::msg::NavVELNED>(std::bind(&DataspeedProduct::callbackNavVelNed, this, std::placeholders::_1), 1);
  }
}

void DataspeedProduct::publishOdom() {
  if (last_hpposllh_.i_tow == last_velned_.i_tow
    && last_hpposllh_.i_tow == last_imu_.i_tow
    && last_hpposllh_.i_tow != odom_itow_) {
      odom_itow_ = last_imu_.i_tow;

      // TODO: Check last_hpposllh_.invalid_llh
      double latitude = last_hpposllh_.lat * 1e-7 + last_hpposllh_.lat_hp * 1e-9;
      double longitude = last_hpposllh_.lon * 1e-7 + last_hpposllh_.lon_hp * 1e-9;
      double utm_x;
      double utm_y;
      std::string utm_zone;
      gps_tools::LLtoUTM(latitude, longitude, utm_y, utm_x, utm_zone);
      if (odom_pub != nullptr) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = imu_msg_.header.stamp;
        odom_msg.child_frame_id = "ins";
        odom_msg.header.frame_id = "UTM_" + utm_zone;
        odom_msg.pose.pose.position.x = utm_x;
        odom_msg.pose.pose.position.y = utm_y;
        odom_msg.pose.pose.position.z = last_hpposllh_.height * 1e-3 + last_hpposllh_.height_hp * 1e-4;

        // Compute convergence angle for current position
        int zone_number;
        char zone_letter;
        std::sscanf(utm_zone.c_str(), "%d%c", &zone_number, &zone_letter);
        double central_meridian = 6 * (zone_number - 1) - 177;
        double convergence_angle = atan(tan(M_PI / 180.0 * (longitude - central_meridian)) * sin(M_PI / 180.0 * latitude));

        // Apply convergence angle correction to ENU orientation quaternion
        tf2::Quaternion q_tf(imu_msg_.orientation.x, imu_msg_.orientation.y, imu_msg_.orientation.z, imu_msg_.orientation.w);
        tf2::Quaternion conv_q(tf2::Vector3(0, 0, 1), convergence_angle);
        q_tf = conv_q * q_tf;
        odom_msg.pose.pose.orientation.x = q_tf.x();
        odom_msg.pose.pose.orientation.y = q_tf.y();
        odom_msg.pose.pose.orientation.z = q_tf.z();
        odom_msg.pose.pose.orientation.w = q_tf.w();

        odom_msg.twist.twist.linear.x = last_velned_.speed * 1e-2;
        odom_msg.twist.twist.angular.z = imu_msg_.angular_velocity.z;
        // TODO: populate covariance fields

        odom_pub->publish(odom_msg);
      }

      if (fix_pub != nullptr) {
        sensor_msgs::msg::NavSatFix fix_msg;
        fix_msg.header.stamp = imu_msg_.header.stamp;
        fix_msg.header.frame_id = frame_id_;
        fix_msg.latitude = latitude;
        fix_msg.longitude = longitude;
        fix_msg.altitude = last_hpposllh_.height * 1e-3 + last_hpposllh_.height_hp * 1e-4;
        fix_msg.status.status = last_hpposllh_.invalid_llh ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix_pub->publish(fix_msg);
      }
  }
}
void DataspeedProduct::callbackNavHpPosLlh(const ublox_msgs::msg::NavHPPOSLLH& m) {
  // Do not publish raw message (already published in HpPosRecProduct callback)
  last_hpposllh_ = m;
  publishOdom();
}
void DataspeedProduct::callbackNavVelNed(const ublox_msgs::msg::NavVELNED& m) {
  // Do not publish raw message
  last_velned_ = m;
  publishOdom();
}
void DataspeedProduct::callbackDsImu(const ublox_msgs::msg::DsIMU& m) {
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = node_->get_clock()->now();
  imu_msg.header.frame_id = "ins";

  if (m.flags & 0x01) { // linear accel valid
    imu_msg.linear_acceleration.x = m.lin_accel_x * 1e-7;
    imu_msg.linear_acceleration.y = m.lin_accel_y * 1e-7;
    imu_msg.linear_acceleration.z = m.lin_accel_z * 1e-7;
    imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = m.l_acc * 1e-7;
  } else {
    imu_msg.linear_acceleration.x = NAN;
    imu_msg.linear_acceleration.y = NAN;
    imu_msg.linear_acceleration.z = NAN;
  }

  if (m.flags & 0x02) { // angular vel valid
    imu_msg.angular_velocity.x = m.ang_vel_x * 5e-6;
    imu_msg.angular_velocity.y = m.ang_vel_y * 5e-6;
    imu_msg.angular_velocity.z = m.ang_vel_z * 5e-6;
    imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = m.a_acc * 5e-6 * M_PI / 180.0;
  } else {
    imu_msg.angular_velocity.x = NAN;
    imu_msg.angular_velocity.y = NAN;
    imu_msg.angular_velocity.z = NAN;
  }

  if (m.flags & 0x04) { // orientation valid
    imu_msg.orientation.w = m.orientation_w;
    imu_msg.orientation.x = m.orientation_x;
    imu_msg.orientation.y = m.orientation_y;
    imu_msg.orientation.z = m.orientation_z;
    imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = imu_msg.orientation_covariance[8] = m.o_acc * 1e-5 * M_PI / 180.0;
  } else {
    imu_msg.orientation.w = NAN;
    imu_msg.orientation.x = NAN;
    imu_msg.orientation.y = NAN;
    imu_msg.orientation.z = NAN;
  }

  if (ds_imu_pub != nullptr) {
    ds_imu_pub->publish(m);
  }

  if (imu_pub != nullptr) {
    imu_pub->publish(imu_msg);
  }

  imu_msg_ = imu_msg;
  last_imu_ = m;
  publishOdom();
}

}