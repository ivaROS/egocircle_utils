#ifndef EGOCIRCLE_UTILS_UPDATEABLE_INTERFACE_H
#define EGOCIRCLE_UTILS_UPDATEABLE_INTERFACE_H

#include <sensor_msgs/LaserScan.h>

namespace egocircle_utils
{
  class UpdateableInterface
  {
    public:
      virtual void update(const sensor_msgs::LaserScan::ConstPtr& scan)=0;
  };

} // end namespace egocircle_utils

#endif //EGOCIRCLE_UTILS_UPDATEABLE_INTERFACE_H
