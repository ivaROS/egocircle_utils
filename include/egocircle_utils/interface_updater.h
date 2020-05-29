#ifndef EGOCIRCLE_UTILS_INTERFACE_UPDATER_H
#define EGOCIRCLE_UTILS_INTERFACE_UPDATER_H

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
    // Use this constructor if you don't have a TransformManager
    InterfaceUpdater(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name=DEFAULT_NAME, const tf2_utils::TransformManager& tfm=tf2_utils::TransformManager(false));
    
    // Use this constructor if you already have a TransformManager
    InterfaceUpdater(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name=DEFAULT_NAME);
    
    void setInterface(std::shared_ptr<UpdateableInterface> interface);
    
    bool init();
    
    void update();
    
    bool isReady();
    
    static constexpr const char* DEFAULT_NAME="egocircle_interface_updater";
    
    std_msgs::Header getCurrentHeader();

  };

}

#endif //EGOCIRCLE_UTILS_INTERFACE_UPDATER_H
