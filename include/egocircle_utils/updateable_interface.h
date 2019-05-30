#ifndef EGOCIRCLE_UTILS_UPDATEABLE_INTERFACE_H
#define EGOCIRCLE_UTILS_UPDATEABLE_INTERFACE_H

#include <pips/collision_testing/transforming_collision_checker.h>
#include <egocircle_utils/transformer.h>
#include <sensor_msgs/LaserScan.h>

namespace egocircle_utils
{

class UpdateableInterface : public pips::collision_testing::TransformingCollisionChecker, public egocircle_utils::Transformer
{
  public:
    UpdateableInterface(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::string& name) :
      pips::collision_testing::TransformingCollisionChecker(nh,pnh,name)
      {}
    
    virtual void update(const sensor_msgs::LaserScan::ConstPtr& scan)=0;
    
    void setTransform(const geometry_msgs::TransformStamped& transform)
    {
      Transformer::setTransform(transform);
    }
    
    
private:
  //egocircle_utils::Transformer transformer_;
  
};

} //namespace

#endif //EGOCIRCLE_UTILS_UPDATEABLE_INTERFACE_H
