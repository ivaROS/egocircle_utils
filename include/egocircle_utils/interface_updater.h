#ifndef EGO_CIRCLE_COST_WRAPPER_H
#define EGO_CIRCLE_COST_WRAPPER_H

// #include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <tf2_utils/transform_manager.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <egocircle_utils/updateable_interface.h>

namespace egocircle_utils
{

class InterfaceUpdater 
{

  
private:
  ros::NodeHandle nh_, pnh_;
  tf2_utils::TransformManager tfm_;
  std::string name_;

  typedef tf2_ros::MessageFilter<sensor_msgs::LaserScan> TF_Filter;
  std::shared_ptr<TF_Filter> scan_tf_filter_;
    
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;

  sensor_msgs::LaserScan::ConstPtr current_scan_;
  
  std::shared_ptr<UpdateableInterface> cc_;
  
protected:
  virtual void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  
public:
  // Use this constructor if you don't have a tfbuffer
  InterfaceUpdater(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));
  
  // Use this constructor if you already have a tfbuffer
  InterfaceUpdater(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);
  
  void setInterface(std::shared_ptr<UpdateableInterface> interface);

  bool init();
  
  void update();
  
  bool isReady();
  
  static constexpr const char* DEFAULT_NAME="egocircle_interface_updater";
  
  std_msgs::Header getCurrentHeader();
  
  // std::shared_ptr<pips::collision_testing::TransformingCollisionChecker> getCC()
  // {
  //   return cc_;
  // }

};

}

#endif //EGO_CIRCLE_COST_WRAPPER_H
