#ifndef EGOCIRCLE_UTILS_INFLATOR_H
#define EGOCIRCLE_UTILS_INFLATOR_H

#include <egocircle_utils/container.h>

namespace egocircle_utils
{

    class Inflator
    {
    public:
      Inflator(const Container& container, float inflation_radius):
        container_(container),
        inflation_radius_(inflation_radius),
        converter_(container.indexer, inflation_radius)
      {
        if(container_.isValid())
        {
          inflated_depths_ = inflate(container_, converter_, inflation_radius_, inflated_point_inds_);
        }
      }
      
      sensor_msgs::LaserScan::Ptr getMsg() const
      {
        sensor_msgs::LaserScan::Ptr inflated_scan = boost::make_shared<sensor_msgs::LaserScan>();
        *inflated_scan = *(container_.scan);
        inflated_scan->ranges = inflated_depths_;
        inflated_scan->range_max = container_.egocircle_radius;
        return inflated_scan;
      }
      
      template<typename T>
      float getRange(T point) const
      {
        int target_ind = container_.indexer.getIndex(point);
        if(target_ind <inflated_depths_.size())
        {
          return inflated_depths_[target_ind];
        }
        else
        {
          ROS_ERROR_STREAM("Target index out of range! Requested index " << target_ind << ", but inflated egocircle only contains " << inflated_depths_.size() << " elements.");
          return -1;
        }
      }
      
      const std::vector<float>& getDepths() const
      {
        return inflated_depths_;
      }
      
      const std::vector<int>& getInds() const
      {
        return inflated_point_inds_;
      }
      
      float getInflationRadius() const
      {
        return inflation_radius_;
      }
      
      const Container& getContainer() const
      {
        return container_;
      }
      
    private:
      const Container& container_;
      float inflation_radius_;
      ego_circle::EgoCircleWidthConverter converter_;
      std::vector<float> inflated_depths_;
      std::vector<int> inflated_point_inds_;

      static std::vector<float> inflate(const Container& container, const ego_circle::EgoCircleWidthConverter& converter, float inflation_radius, std::vector<int>& inflated_point_inds)
      {
        ros::WallTime start = ros::WallTime::now();
        std::vector<float> inflated_depths = container.depths; //(depths.size());
        const std::vector<float>& depths = container.depths;
        float egocircle_radius = container.egocircle_radius;
        
        inflated_point_inds.resize(depths.size(), -1);
        std::vector<int> ns(depths.size());
        for(int i = 0; i < depths.size(); i++)
        {
          ns[i] = converter.getN(depths[i]);
        }
        int n;
        for(int i = 0; i < depths.size(); i++)
        {
          n = ns[i];
          
          for(int j = i - n; j < i + n; j++)
          {
            int ind = (j <0) ? depths.size() + j : ((j >= depths.size()) ? j - depths.size() : j );
            float cur_inf_depth = inflated_depths[ind];
            float new_inf_depth = depths[i] -inflation_radius;
            if(new_inf_depth < cur_inf_depth)
            {
              inflated_depths[ind] = new_inf_depth;
              inflated_point_inds[ind] = ind;
            }
            //inflated_depths[ind] = std::min(depths[i] -inflation_radius,inflated_depths[ind]);
          }
          
        }
        
        for(int i = 0; i < depths.size(); i++)
        {
          if(inflated_depths[i] == egocircle_radius - inflation_radius)
          {
            inflated_depths[i] = egocircle_radius;
          }
        }
        
        ROS_DEBUG_STREAM_NAMED("timing", "Time to inflate=" << (ros::WallTime::now()-start).toSec()*1000 << "ms");
        
        return inflated_depths;
      }
      
      
      static std::vector<float> inflate(const Container& container, const ego_circle::EgoCircleWidthConverter& converter, float inflation_radius)
      {
        ros::WallTime start = ros::WallTime::now();
        std::vector<float> inflated_depths = container.depths; //(depths.size());
        const std::vector<float>& depths = container.depths;
        float egocircle_radius = container.egocircle_radius;
        
        std::vector<int> ns(depths.size());
        for(int i = 0; i < depths.size(); i++)
        {
          ns[i] = converter.getN(depths[i]);
        }
        int n;
        for(int i = 0; i < depths.size(); i++)
        {
          n = ns[i];
          
          for(int j = i - n; j < i + n; j++)
          {
            int ind = (j <0) ? depths.size() + j : ((j >= depths.size()) ? j - depths.size() : j );
            inflated_depths[ind] = std::min(depths[i] -inflation_radius,inflated_depths[ind]);
          }
          
        }
        
        for(int i = 0; i < depths.size(); i++)
        {
          if(inflated_depths[i] == egocircle_radius - inflation_radius)
          {
            inflated_depths[i] = egocircle_radius;
          }
        }
        
        ROS_DEBUG_STREAM_NAMED("timing", "Time to inflate=" << (ros::WallTime::now()-start).toSec()*1000 << "ms");
        
        return inflated_depths;
      }
    
    
   };
} //namespace

#endif //EGOCIRCLE_UTILS_INFLATOR_H
