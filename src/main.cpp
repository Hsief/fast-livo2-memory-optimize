#include "LIVMapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fast_livo2");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  LIVMapper mapper(nh);
  mapper.initializeSubscribersAndPublishers(nh, it);

  // Sensor callbacks/preprocessing run independently from the estimator loop.
  // roscpp preserves callback serialization within each subscription, so
  // LiDAR preprocessing itself is not executed concurrently with another
  // LiDAR callback.
  ros::AsyncSpinner spinner(3);
  spinner.start();
  mapper.run();
  spinner.stop();
  return 0;
}