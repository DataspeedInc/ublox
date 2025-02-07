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
  if (getRosBoolean(node_, "publish.nav.pvt")) {
    RCLCPP_FATAL(node_->get_logger(), "!!!!");
  }

  (void)frame_id;
  if (getRosBoolean(node_, "publish.ds.odom") || getRosBoolean(node_, "publish.ds.all")) {
    odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  }
  if (getRosBoolean(node_, "publish.ds.imu") || getRosBoolean(node_, "publish.ds.all")) {
    imu_pub = node_->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    body_vel_pub = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("body_vel", 1);
    ds_imu_pub = node_->create_publisher<ublox_msgs::msg::DsIMU>("dsimu", 1);
  }
  if (getRosBoolean(node_, "publish.ds.fix") || getRosBoolean(node_, "publish.ds.all")) {
    fix_pub = node_->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 1);
    time_pub = node_->create_publisher<ublox_msgs::msg::NavTIMEGPS>("timegps", 1);
    dop_pub = node_->create_publisher<ublox_msgs::msg::NavDOP>("dop", 1);
  }
}

void DataspeedProduct::subscribe(std::shared_ptr<ublox_gps::Gps> gps) {
  if (getRosBoolean(node_, "publish.ds.imu") || getRosBoolean(node_, "publish.ds.odom") || getRosBoolean(node_, "publish.ds.all")) {
    gps->subscribe<ublox_msgs::msg::DsIMU>(std::bind(&DataspeedProduct::callbackDsImu, this, std::placeholders::_1), 1);
  }
  if (getRosBoolean(node_, "publish.ds.fix") || getRosBoolean(node_, "publish.ds.odom") || getRosBoolean(node_, "publish.ds.all")) {
    gps->subscribe<ublox_msgs::msg::NavPVT>(std::bind(&DataspeedProduct::callbackNavPvt, this, std::placeholders::_1), 1);
    gps->subscribe<ublox_msgs::msg::NavTIMEGPS>(std::bind(&DataspeedProduct::callbackNavTimeGps, this, std::placeholders::_1), 1);
    gps->subscribe<ublox_msgs::msg::NavDOP>(std::bind(&DataspeedProduct::callbackNavDop, this, std::placeholders::_1), 1);
  }
}

void DataspeedProduct::publishOdom() {
  odom_itow_ = last_imu_.i_tow;

  // TODO: Check last_hpposllh_.invalid_llh
  double latitude = last_pvt_.lat * 1e-7;
  double longitude = last_pvt_.lon * 1e-7;
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
    odom_msg.pose.pose.position.z = last_pvt_.height * 1e-3;

    // Compute convergence angle for current position
    int zone_number;
    char zone_letter;
    std::sscanf(utm_zone.c_str(), "%d%c", &zone_number, &zone_letter);
    double central_meridian = 6 * (zone_number - 1) - 177;
    double convergence_angle = atan(tan(M_PI / 180.0 * (longitude - central_meridian)) * sin(M_PI / 180.0 * latitude));

    // Apply convergence angle to ENU heading to obtain UTM orientation
    tf2::Quaternion q_enu(tf2::Vector3(0, 0, 1), (90.0 - last_pvt_.head_veh * 1e-5) * M_PI / 180.0);
    tf2::Quaternion conv_q(tf2::Vector3(0, 0, 1), convergence_angle);
    auto q_utm = conv_q * q_enu;
    odom_msg.pose.pose.orientation.x = q_utm.x();
    odom_msg.pose.pose.orientation.y = q_utm.y();
    odom_msg.pose.pose.orientation.z = q_utm.z();
    odom_msg.pose.pose.orientation.w = q_utm.w();

    odom_msg.twist.twist.linear.x = last_pvt_.g_speed * 1e-3;
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
    fix_msg.altitude = last_pvt_.height * 1e-3;
    fix_msg.status.status = (last_pvt_.flags & ublox_msgs::msg::NavPVT::FLAGS_GNSS_FIX_OK) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix_pub->publish(fix_msg);
  }
}
void DataspeedProduct::callbackNavPvt(const ublox_msgs::msg::NavPVT& m) {
  last_pvt_ = m;
}
void DataspeedProduct::callbackNavDop(const ublox_msgs::msg::NavDOP& m) {
  // dop_pub->publish(m);
}
void DataspeedProduct::callbackNavTimeGps(const ublox_msgs::msg::NavTIMEGPS& m) {
  // time_pub->publish(m);
}
void DataspeedProduct::callbackDsImu(const ublox_msgs::msg::DsIMU& m) {
  sensor_msgs::msg::Imu imu_msg;
  geometry_msgs::msg::Vector3Stamped body_vel_msg;
  imu_msg.header.stamp = body_vel_msg.header.stamp = node_->get_clock()->now();
  imu_msg.header.frame_id = body_vel_msg.header.frame_id = "ins";

  if (m.flags & 0x01) { // linear accel valid
    imu_msg.linear_acceleration.x = m.accel_x * 1e-7;
    imu_msg.linear_acceleration.y = m.accel_y * 1e-7;
    imu_msg.linear_acceleration.z = m.accel_z * 1e-7;
    imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = 0.0;
  } else {
    imu_msg.linear_acceleration.x = NAN;
    imu_msg.linear_acceleration.y = NAN;
    imu_msg.linear_acceleration.z = NAN;
  }

  if (m.flags & 0x02) { // angular vel valid
    imu_msg.angular_velocity.x = m.gyro_x * 5e-6;
    imu_msg.angular_velocity.y = m.gyro_y * 5e-6;
    imu_msg.angular_velocity.z = m.gyro_z * 5e-6;
    imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = 0;
  } else {
    imu_msg.angular_velocity.x = NAN;
    imu_msg.angular_velocity.y = NAN;
    imu_msg.angular_velocity.z = NAN;
  }

  if (m.flags & 0x03) { // linear vel valid
    body_vel_msg.vector.x = m.vel_x * 5e-6;
    body_vel_msg.vector.y = m.vel_y * 5e-6;
    body_vel_msg.vector.z = m.vel_z * 5e-6;
  } else {
    body_vel_msg.vector.x = NAN;
    body_vel_msg.vector.y = NAN;
    body_vel_msg.vector.z = NAN;
  }

  if (m.flags & 0x08) { // orientation valid
    tf2::Quaternion q;
    q.setRPY(m.roll * M_PI / 180.0 * 1e-5, m.pitch * M_PI / 180.0 * 1e-5, m.heading * M_PI / 180.0 * 1e-5);
    imu_msg.orientation.w = q.w();
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation_covariance[0] = (m.roll_acc * 1e-6) * (m.roll_acc * 1e-6);
    imu_msg.orientation_covariance[4] = (m.pitch_acc * 1e-6) * (m.pitch_acc * 1e-6);
    imu_msg.orientation_covariance[8] = (m.heading_acc * 1e-6) * (m.heading_acc * 1e-6);
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