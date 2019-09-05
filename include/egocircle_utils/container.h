#ifndef EGOCIRCLE_UTILS_CONTAINER_H
#define EGOCIRCLE_UTILS_CONTAINER_H

#include <egocircle/ego_circle.h>

namespace egocircle_utils
{
    struct Container
    {
      Container(const sensor_msgs::LaserScan::ConstPtr& scan):
        scan(scan),
        depths(scan->ranges),
        indexer(depths.size()),
        egocircle_radius(scan->range_max-ego_circle::EgoCircleROS::OFFSET) //NOTE: specified radius of scan is larger than the egocircle radius so that the border of the egocircle is visible
      {
      }
      
      bool isValid() const
      {
        return depths.size()>0;
      }
      
      template <typename T>
      float getRange(T point) const
      {
        int target_ind = indexer.getIndex(point);
        if(target_ind <depths.size())
        {
          return depths[target_ind];
        }
        else
        {
          ROS_ERROR_STREAM("Target index out of range! Requested index " << target_ind << ", but egocircle only contains " << depths.size() << " elements.");
          return -1;
        }
      }
      
      sensor_msgs::LaserScan::ConstPtr scan;
      const std::vector<float>& depths;
      float egocircle_radius;
      ego_circle::EgoCircleIndexer indexer;
    };

} //namespace

#endif //EGOCIRCLE_UTILS_CONTAINER_H
