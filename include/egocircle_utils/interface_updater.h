#ifndef EGO_CIRCLE_COST_WRAPPER_H
#define EGO_CIRCLE_COST_WRAPPER_H

#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <message_filters/subscriber.h>
#include <egocircle_utils/updateable_interface.h>

namespace egocircle_utils
{

class InterfaceUpdater : public pips_trajectory_testing::PipsCCWrapper
{

  
private:
    
  typedef tf2_ros::MessageFilter<sensor_msgs::LaserScan> TF_Filter;
  std::shared_ptr<TF_Filter> scan_tf_filter_;
    
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::LaserScan::ConstPtr current_scan_;
  
  
  static constexpr const int MAGIC_NUMBER = -17;  //The sole purpose of this is to ensure that the constructor with the 'optional' tfbuffer argument is not accidentally passed a tfbuffer object
  
  std::shared_ptr<UpdateableInterface> cc_;
    
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  
  
  
public:
  // Use this constructor if you don't have a tfbuffer
  InterfaceUpdater(std::shared_ptr<UpdateableInterface> interface, ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, int tamper_prevention = MAGIC_NUMBER, std::shared_ptr<tf2_ros::Buffer> tf_buffer=std::make_shared<tf2_ros::Buffer>());
  
  // Use this constructor if you already have a tfbuffer
  InterfaceUpdater(std::shared_ptr<UpdateableInterface> interface, ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string& name=DEFAULT_NAME);
  
  
  bool init();
  
  void update();
  
  bool isReadyImpl();
  
  static constexpr const char* DEFAULT_NAME="egocircle_interface_updater";
  
  std_msgs::Header getCurrentHeader();
  
  std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
  {
    return cc_;
  }

};

}

#endif //EGO_CIRCLE_COST_WRAPPER_H
