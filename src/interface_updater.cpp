#include <egocircle_utils/interface_updater.h>

namespace egocircle_utils
{

  InterfaceUpdater::InterfaceUpdater(std::shared_ptr<UpdateableInterface> interface, ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const int tamper_prevention, std::shared_ptr<tf2_ros::Buffer> tf_buffer) :
    PipsCCWrapper(nh,pnh,name,tf_buffer),
    cc_(interface)
{
    // If a tfbuffer was not provided by the user, then we need to set up a listener
    if(tamper_prevention == MAGIC_NUMBER)
    {
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }
}


InterfaceUpdater::InterfaceUpdater(std::shared_ptr<UpdateableInterface> interface, ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const std::string& name) :
    PipsCCWrapper(nh,pnh,name, tf_buffer),
    cc_(interface)
{
}

bool InterfaceUpdater::init()
{
  if(!inited_)
  {
    PipsCCWrapper::init();
  
    // Get topic names
    std::string ego_circle_topic="/point_scan";
    
    pnh_.getParam("ego_circle_topic", ego_circle_topic );
    
    // The idea here is to set the parameter on the parameter server to the default value to make it easier to see what it is.
    pnh_.setParam("ego_circle_topic", ego_circle_topic);

    std::string odom_frame_id = "odom";

    //NOTE: It isn't clear to me whether it matters if I use pnh_ or nh_
    std::string key;
    if (pnh_.searchParam("odom_frame_id", key))
    {
      pnh_.getParam(key, odom_frame_id );
      pnh_.setParam(key, odom_frame_id );
    }
    
    ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

    
    scan_sub_.subscribe ( nh_, ego_circle_topic, 3 );    
    
    // Ensure that scan is transformable
    scan_tf_filter_ = std::make_shared<TF_Filter>(scan_sub_, *tf_buffer_, odom_frame_id, 2,nh_);
    
    scan_tf_filter_->registerCallback(boost::bind(&InterfaceUpdater::scanCb, this, _1));
    
    inited_ = true;
  }
    return true;

}


void InterfaceUpdater::update()
{
    ROS_DEBUG_STREAM_NAMED ( name_, "Updating collision checker laserscan" );
    
    if(current_scan_)
    {
      cc_->update( current_scan_);
    }
    else
    {
      ROS_WARN_STREAM("No laserscan received!");
    }
}

bool InterfaceUpdater::isReadyImpl()
{
  if(current_scan_)
    {
      return true;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(name_, "No current LaserScan message!");
    }
    return false;
}

    
void InterfaceUpdater::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    if ( scan_msg->header.stamp == ros::Time ( 0 )) { // Gazebo occasionally publishes Image and CameraInfo messages with time=0
        ROS_WARN_STREAM_NAMED ( name_,"Bad timestamp" );
        return;
    }

    {
      current_scan_ = scan_msg;
    }

    doCallback();
}

std_msgs::Header InterfaceUpdater::getCurrentHeader()
{
    if(current_scan_)
    {
      return current_scan_->header;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(name_, "Trying to get current header, but no scan message has been received!");
        return std_msgs::Header();  //TODO: Add a warning here; this really shouldn't happen
    }
}

}
