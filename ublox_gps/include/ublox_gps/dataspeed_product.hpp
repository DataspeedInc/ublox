#ifndef DATASPEED_PRODUCT_HPP
#define DATASPEED_PRODUCT_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ublox_gps/gps.hpp>
#include <ublox_gps/hp_pos_rec_product.hpp>
#include <ublox_gps/rtcm.hpp>

#include <ublox_msgs/msg/nav_pvt.hpp>

namespace ublox_node {

/**
 *  @brief TODO
 */
class DataspeedProduct final : public virtual HpgRefProduct {
 public:
  explicit DataspeedProduct(uint16_t nav_rate, uint16_t meas_rate, const std::string & frame_id, std::shared_ptr<diagnostic_updater::Updater> updater, std::vector<ublox_gps::Rtcm> rtcms, rclcpp::Node* node);


  /**
   * @brief Subscribe to Rover messages, such as NavRELPOSNED.
   */
  void subscribe(std::shared_ptr<ublox_gps::Gps> gps) override;

 protected:

  /**
   * @brief TODO
   * @param m UBX message structure
   */
  void callbackNavPvt(const ublox_msgs::msg::NavPVT& m);

  /**
   * @brief TODO
   * @param m UBX message structure
   */
  void callbackNavDop(const ublox_msgs::msg::NavDOP& m);

  /**
   * @brief TODO
   * @param m UBX message structure
   */
  void callbackNavTimeGps(const ublox_msgs::msg::NavTIMEGPS& m);

  /**
   * @brief Publish a sensor_msgs/Imu message upon receiving a DSIMU UBX message
   * @param m UBX message structure
   */
  void callbackDsImu(const ublox_msgs::msg::DsIMU& m);

  /**
   * @brief TODO
   */
  void publishOdom();

 protected:
  uint32_t odom_itow_ = 0;
  sensor_msgs::msg::Imu imu_msg_;
  ublox_msgs::msg::NavPVT last_pvt_;
  ublox_msgs::msg::DsIMU last_imu_;
  std::string frame_id_;

 private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr body_vel_pub;
  rclcpp::Publisher<ublox_msgs::msg::DsIMU>::SharedPtr ds_imu_pub;
  rclcpp::Publisher<ublox_msgs::msg::NavDOP>::SharedPtr dop_pub;
  rclcpp::Publisher<ublox_msgs::msg::NavTIMEGPS>::SharedPtr time_pub;
};

}  // namespace ublox_node

#endif // DATASPEED_PRODUCT_HPP
