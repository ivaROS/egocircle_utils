#ifndef EGOCIRCLE_UTILS_MIN_DIST_H
#define EGOCIRCLE_UTILS_MIN_DIST_H


#include <egocircle_utils/container.h>

namespace egocircle_utils
{
  
  class PolarDistanceCalculator
  {
  public:
    
    //TODO: precompute relevant info
    PolarDistanceCalculator(const Container& container):
      ranges_(container.depths),
      angle_increment_(container.scan->angle_increment),
      egocircle_radius_(container.egocircle_radius)
    {
      //       float angle = scan->angle_min
      //       int num_deltas = scan->ranges.size()/2;
      //       coeffs_.resize(num_deltas);
      //       ranges_sq_.resize(scan->ranges.size());
      //       
      //       for(int i = 0; i < num_deltas; ++i, angle+=scan->angle_increment)
      //       {
      //         
      //       }
      //       
      //       for(int i = 0; i < scan->ranges.size(); ++i)
      //       {
      //         ranges_sq_[i] = scan->ranges[i] * scan->ranges[i];
      //       }
    }
    
    float getDistanceSq(int target_ind, float target_depth, int point_ind) const
    {
      float point_depth = ranges_[point_ind];
      if(point_depth == egocircle_radius_)
      {
        return std::numeric_limits<float>::max();
      }
      
      float d2 = point_depth*point_depth + target_depth*target_depth - 2 * point_depth*target_depth*std::cos(angle_increment_*(target_ind - point_ind));
      return d2;
    }
    
    float getDistanceSq(int ind1, int ind2) const
    {
      if(ind1==ind2)
        return 0; //I wonder if the compiler would have deduced this on its own...
        
        float range1 = ranges_[ind1];
      if(range1 == egocircle_radius_)
      {
        return std::numeric_limits<float>::max();
      }
      
      return getDistanceSq(ind1, range1, ind2);
    }
    
  private:
    //     std::vector<float> coeffs_;
    //     std::vector<float> ranges_sq_;
    const std::vector<float>& ranges_;
    float angle_increment_, egocircle_radius_;
  };

  class MinDistanceCalculator
  {
  public:
    MinDistanceCalculator(const Container& container, float search_radius):
      container_(container),
      converter_(container_.indexer, search_radius),
      polar_distance_calculator_(container_)
    {
      if(!container.isValid())
      {
        ROS_WARN_DELAYED_THROTTLE(1,"[MinDistanceCalculator] Container is empty! This is only acceptable during initial construction");
      }
    }
    

    float getMinDist(ego_circle::EgoCircularPoint point) const
    {
      float target_depth = std::sqrt(point.getKey());
      int target_ind = container_.indexer.getIndex(point);

      int size = container_.depths.size();
      
      int n = converter_.getN(target_depth);
      n = n % (size/2);
      float min_depth_sq = std::numeric_limits<float>::max();
      
      for(int j = target_ind - n; j < target_ind + n; j++)
      {
        int ind = (j <0) ? size + j : ((j >= size) ? j - size : j );
        float dist_sq = polar_distance_calculator_.getDistanceSq(target_ind, target_depth, ind);
        
        //if(dist_sq > 100)
        //  ROS_WARN_STREAM("j=" << j << ", ind=" << ind << ", dist_sq=" << dist_sq << ", prev min_depth_sq=" << min_depth_sq);
        
        min_depth_sq = std::min(dist_sq,min_depth_sq);
      }
      
      float min_dist = std::sqrt(min_depth_sq);
      
      //"Transform: " << toString(current_transform_) << 
      ROS_DEBUG_STREAM_NAMED("MinDistanceCalculator","point = (" << point.x << "," << point.y << "), depth = " << target_depth << ", index=" << target_ind << ", width=" << n << ", min_depth_sq = " << min_depth_sq << ", min_dist = " << min_dist);
      
      return min_dist;
    }
    
  private:
    const Container& container_;
    ego_circle::EgoCircleWidthConverter converter_;
    PolarDistanceCalculator polar_distance_calculator_;

  };

} //namespace

#endif //EGOCIRCLE_UTILS_MIN_DIST_H
