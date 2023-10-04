#include <egocircle_utils/interface_updater.h>

namespace egocircle_utils
{

  InterfaceUpdater::InterfaceUpdater(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name, const tf2_utils::TransformManager& tfm) :
    nh_(nh),
    pnh_(pnh, name),
    tfm_(tfm,nh),
    name_(name)
  {}
  
  
  InterfaceUpdater::InterfaceUpdater(ros::NodeHandle& nh, ros::NodeHandle& pnh, const tf2_utils::TransformManager& tfm, const std::string& name) :
    nh_(nh),
    pnh_(pnh, name),
    tfm_(tfm,nh),
    name_(name)
  {}

void InterfaceUpdater::setInterface(std::shared_ptr<UpdateableInterface> interface)
{
  cc_ = interface;
}

bool InterfaceUpdater::init()
{
  // Get topic names
  std::string ego_circle_topic="/point_scan";
  
  std::string topic_key;
  if (pnh_.searchParam("ego_circle_topic", topic_key))
  {
    pnh_.getParam(topic_key, ego_circle_topic );
  }
  else
  {
    ROS_WARN_STREAM("No value found for parameter [ego_circle_topic], using default value [" << ego_circle_topic << "]");
  }
  // if(!pnh_.getParam("ego_circle_topic", ego_circle_topic ))
  // {
  //   ROS_WARN_STREAM("No value found for parameter [ego_circle_topic], using default value [" << ego_circle_topic << "]");
  //   // The idea here is to set the parameter on the parameter server to the default value to make it easier to where it is
  //   pnh_.setParam("ego_circle_topic", ego_circle_topic);
  // }

  std::string odom_frame_id = "odom";

  //NOTE: It isn't clear to me whether it matters if I use pnh_ or nh_
  std::string key;
  if (pnh_.searchParam("odom_frame_id", key))
  {
    pnh_.getParam(key, odom_frame_id );
    // pnh_.setParam(key, odom_frame_id );
  }
  else
  {
    ROS_WARN_STREAM("No value found for parameter [odom_frame_id], using default value [" << odom_frame_id << "]");
  }
  
  ROS_DEBUG_STREAM_NAMED ( name_,  "Setting up publishers and subscribers" );

  
  scan_sub_.subscribe ( nh_, ego_circle_topic, 3 );    
  
  // Ensure that scan is transformable
  scan_tf_filter_ = std::make_shared<TF_Filter>(scan_sub_, *tfm_.getBuffer(), odom_frame_id, 2,nh_);
  scan_tf_filter_->registerCallback(boost::bind(&InterfaceUpdater::scanCb, this, _1));

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

bool InterfaceUpdater::isReady()
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
