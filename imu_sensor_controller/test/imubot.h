#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_buffer.h>

#include <limits>
#include <sstream>

template <unsigned int NUM_SENSORS = 1>
class Imubot : public hardware_interface::RobotHW
{
  public:
    Imubot()
    : running_(true)
    , start_srv_(nh_.advertiseService("start", &Imubot::start_callback, this))
    , stop_srv_(nh_.advertiseService("stop", &Imubot::stop_callback, this))
    {
      std::fill(ori_[0], ori_[NUM_SENSORS], 0.0);
      std::fill(avl_[0], avl_[NUM_SENSORS], 0.0);
      std::fill(lac_[0], lac_[NUM_SENSORS], 0.0);
      std::fill(ori_cov_[0], ori_cov_[NUM_SENSORS], 0.0);
      std::fill(avl_cov_[0], avl_cov_[NUM_SENSORS], 0.0);
      std::fill(lac_cov_[0], lac_cov_[NUM_SENSORS], 0.0);

      for (unsigned int i = 0; i < NUM_SENSORS; ++i)
      {
        std::ostringstream os;
	os << "imu_sensor_" << i;

	hardware_interface::ImuSensorHandle::Data data;
	data.name = os.str();
	data.orientation         = ori_[i];
	data.angular_velocity    = avl_[i];
	data.linear_acceleration = lac_[i];
	data.orientation_covariance         = ori_cov_[i];
	data.angular_velocity_covariance    = avl_cov_[i];
	data.linear_acceleration_covariance = lac_cov_[i];

        hardware_interface::ImuSensorHandle imu_sensor_handle_ = hardware_interface::ImuSensorHandle(data);
	imu_sensor_interface_.registerHandle(imu_sensor_handle_);
      }
      registerInterface(&imu_sensor_interface_);
    }

    ros::Time getTime() const { return ros::Time::now(); }
    ros::Duration getPeriod() const { return ros::Duration(0.01); }

    void read()
    {
      ROS_INFO_STREAM("read");
    }

    void write()
    {
      ROS_INFO_STREAM("write");
    }

    bool start_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
      running_ = true;
      return true;
    }

    bool stop_callback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
      running_ = false;
      return true;
    }

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer start_srv_;
    ros::ServiceServer stop_srv_;
    bool running_;

    hardware_interface::ImuSensorInterface imu_sensor_interface_;

    double ori_[NUM_SENSORS][4];
    double avl_[NUM_SENSORS][3];
    double lac_[NUM_SENSORS][3];
    double ori_cov_[NUM_SENSORS][9];
    double avl_cov_[NUM_SENSORS][9];
    double lac_cov_[NUM_SENSORS][9];
};
