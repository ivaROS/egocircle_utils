#ifndef EGOCIRCLE_UTILS_TRANSFORMER_H
#define EGOCIRCLE_UTILS_TRANSFORMER_H

#include <egocircle/ego_circle.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pips/utils/pose_conversions.h>

namespace egocircle_utils
{
  


  class Transformer
  {
  public:
    
    static std::string toString(tf2::Transform transform)
    {
      
      auto origin = transform.getOrigin();
      auto m_el = transform.getBasis();
      std::stringstream ss;
      ss << "(" <<
      tf2Scalar(origin.x())<< ", " <<
      tf2Scalar(origin.y())<< ", " <<
      tf2Scalar(origin.z())<< ") " <<
      "[" << 
      tf2Scalar(m_el.getRow(0).x())<< ", " <<
      tf2Scalar(m_el.getRow(1).x())<< ", " <<
      tf2Scalar(m_el.getRow(2).x())<< ", " <<
      tf2Scalar(0.0)<< ", " <<
      tf2Scalar(m_el.getRow(0).y())<< ", " <<
      tf2Scalar(m_el.getRow(1).y())<< ", " <<
      tf2Scalar(m_el.getRow(2).y())<< ", " <<
      tf2Scalar(0.0)<< ", " <<
      tf2Scalar(m_el.getRow(0).z())<< ", " <<
      tf2Scalar(m_el.getRow(1).z())<< ", " <<
      tf2Scalar(m_el.getRow(2).z())<< ", " <<
      tf2Scalar(0.0)<< "]";
      
      return ss.str();
      
    }
    

//     ego_circle::EgoCircularPoint toLocal(ego_circle::EgoCircularPoint&& point) const
//     {
//       EgoCircularPoint transformed_point = point;
//       ego_circle::applyTransform(transformed_point, current_transform_);
//     }
    
    void toLocal(ego_circle::EgoCircularPoint& point) const
    {
      ego_circle::applyTransform(point, current_transform_);
    }
    
    void toGlobal(ego_circle::EgoCircularPoint& point) const
    {
      ego_circle::applyTransform(point, inv_current_transform_);
    }
    
    void toGlobal(std::vector<ego_circle::EgoCircularPoint>& points) const
    {
      for(ego_circle::EgoCircularPoint& point : points)
      {
        toGlobal(point);
      }
    }
    
    std::string getLocalFrameId()
    {
      return transform_msg_.header.frame_id;
    }
    
    std::string getGlobalFrameId()
    {
      return transform_msg_.child_frame_id;
    }
    
    ros::Time getStamp()
    {
      return transform_msg_.header.stamp;
    }
    
    void setTransform(const geometry_msgs::TransformStamped& transform)
    {
      transform_msg_ = transform;
      
      current_transform_ = ego_circle::SE2Transform(transform);
      
      tf2::Stamped<tf2::Transform> tf2_trans;
      tf2::fromMsg(transform, tf2_trans);
      
      ROS_DEBUG_STREAM("original transform: " << toString((tf2::Transform)tf2_trans));
      
      tf2::Transform tf2_inv = tf2_trans.inverse();
      
      ROS_DEBUG_STREAM("inverse transform: " << toString((tf2::Transform)tf2_inv));
      
      tf2::Stamped<tf2::Transform> tf2_inv_stamped;
      tf2_inv_stamped.setData(tf2_inv);
      
      geometry_msgs::TransformStamped inv_transform_stamped;
      inv_transform_stamped = tf2::toMsg(tf2_inv_stamped);
      
      ROS_DEBUG_STREAM("original transform stamped: " << ::toString(transform));
      
      ROS_DEBUG_STREAM("inverse transform stamped: " << ::toString(inv_transform_stamped));
      
      inv_current_transform_ = ego_circle::SE2Transform(inv_transform_stamped);
    }
    
  private:
    
    ego_circle::SE2Transform current_transform_, inv_current_transform_;
    geometry_msgs::TransformStamped transform_msg_;

  };

} //namespace

#endif //
