#ifndef EGOCIRCLE_UTILS_DECIMATOR_H
#define EGOCIRCLE_UTILS_DECIMATOR_H

#include <egocircle_utils/container.h>

namespace egocircle_utils
{
  
  class Decimator
  {
    
  public:
    Decimator(const Container& container, float inscribed_radius):
      container_(container),
      inscribed_radius_(inscribed_radius)
    {
      if(container_.isValid())
      {
        decimated_points_ = getDecimatedEgoCircularPoints(container_, inscribed_radius_);
      }
    }
    
    visualization_msgs::Marker::Ptr getMsg(std::string ns="decimated_scan") const
    {
      visualization_msgs::Marker::Ptr decimated_scan = boost::make_shared<visualization_msgs::Marker>();
      decimated_scan->ns=ns;
      decimated_scan->id=0;
      decimated_scan->type=visualization_msgs::Marker::SPHERE_LIST;
      decimated_scan->action=visualization_msgs::Marker::ADD;
      
      decimated_scan->header=container_.scan->header;
      decimated_scan->color.a=.5;
      decimated_scan->color.r=1;
      decimated_scan->color.g=1;
      decimated_scan->color.b=0;
      decimated_scan->scale.x=inscribed_radius_*2;//0.01;
      decimated_scan->scale.y=inscribed_radius_*2;//0.01;
      decimated_scan->scale.z=inscribed_radius_*2;//1;
      
      for(ego_circle::EgoCircularPoint point : decimated_points_)
      {
        geometry_msgs::Point point_msg;
        point_msg.x=point.x;
        point_msg.y=point.y;
        
        decimated_scan->points.push_back(point_msg);
      }
      
      return decimated_scan;
    }
    
    const std::vector<ego_circle::EgoCircularPoint>& getPoints() const
    {
      return decimated_points_;
    }
    
//     float getInflationRadius() const
//     {
//       return inflation_radius_;
//     }
//     
//     const Container& getContainer() const
//     {
//       return container_;
//     }
    
  private:
    static std::vector<ego_circle::EgoCircularPoint> getDecimatedEgoCircularPoints(const Container& container, const float inscribed_radius)
    {
      using ego_circle::EgoCircularPoint;
      using ego_circle::PolarPoint;
            
      const sensor_msgs::LaserScan& scan = *container.scan;
      const float angle_increment = scan.angle_increment;
      float current_angle = scan.angle_min;
      
      const int num_points = scan.ranges.size();
      
      const float r2 = inscribed_radius*inscribed_radius;
      
      const float boundary_radius = container.egocircle_radius;
                  
      std::vector<EgoCircularPoint> points;
      
      PolarPoint target_point(-1,current_angle);
      PolarPoint prev_point(-1,current_angle);
      
      for(int i = 0; i < num_points; ++i)
      {
        PolarPoint polar_point(scan.ranges[i],current_angle);
        
        if(polar_point.r < boundary_radius)
        {
          bool use_point = false;
          if(target_point.r<0)
          {
            use_point=true;
          }
          
          else
          {
            float d2 = polar_point.r*polar_point.r + target_point.r*target_point.r - 2 * polar_point.r*target_point.r*std::cos(current_angle - target_point.theta);
            
            if(d2>r2)
            {
              use_point=true;
              if(prev_point.r>0)
              {
                float prev_d2 = polar_point.r*polar_point.r + prev_point.r*prev_point.r - 2 * polar_point.r*prev_point.r*std::cos(current_angle - prev_point.theta);
                if(prev_d2>r2)
                {
                  points.push_back(prev_point);
                  prev_point.r=-1;
                }
              }
            }
          }
          
          if(use_point)
          {
            target_point = polar_point;
            
            points.push_back(target_point);
          }
          else
          {
            prev_point=polar_point;
          }
        }
        current_angle += angle_increment;
        
      }
      if(prev_point.r>0)
      {
        points.push_back(prev_point);
      }
      
      return points;
    }

    
    const Container& container_;
    float inscribed_radius_;
    std::vector<ego_circle::EgoCircularPoint> decimated_points_;
    
  };
} //namespace

#endif //EGOCIRCLE_UTILS_DECIMATOR_H
