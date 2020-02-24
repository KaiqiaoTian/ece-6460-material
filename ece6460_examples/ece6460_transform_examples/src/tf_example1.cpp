// Include the main header for the TF library
#include <tf/tf.h>

// Terminal IO
#include <stdio.h>

int main(int argc, char** argv)
{
  tf::Vector3 vehicle_position(10.0, 3.0, 0.0);
  tf::Vector3 relative_position(25.0, 0.0, 0.0);
  
  double theta = 120.0 * M_PI / 180.0;
  tf::Quaternion q = tf::createQuaternionFromYaw(theta);
  
  tf::Transform transform;
  transform.setOrigin(vehicle_position);
  transform.setRotation(q);
  
  tf::Vector3 target_position = transform * relative_position;
  printf("Target position: (%f, %f, %f)\n", target_position.x(), target_position.y(), target_position.z());
  
  return 0;
}