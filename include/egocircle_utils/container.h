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
        ego_circle::LaserScanWrapper it_scan(*scan);
        
        for(auto pnt : it_scan)
        {
          points.push_back(pnt);
        }
      }
      
      bool isValid() const
      {
        return depths.size()>0;
      }
      
      template <typename T>
      float getRange(T point) const
      {
        unsigned int target_ind = indexer.getIndex(point);
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
      std::vector<ego_circle::PolarPoint> points;
      ego_circle::EgoCircleIndexer indexer;
      float egocircle_radius;
    };

} //namespace

#endif //EGOCIRCLE_UTILS_CONTAINER_H
