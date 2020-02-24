// Include the main header for the TF library
#include <tf/tf.h>

// Terminal IO
#include <stdio.h>

int main(int argc, char** argv)
{
  tf::Vector3 vehicle_position(14.0, 14.0, 0.0);
  tf::Vector3 target_location(5.0, 27.0, 0.0);
  
  double theta = 120.0 * M_PI / 180.0;
  
  tf::Transform transform;
  transform.setOrigin(vehicle_position);
  tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
  transform.setRotation(quat);
  
  tf::Transform inv_transform = transform.inverse();
  
  tf::Vector3 inv_translation = inv_transform.getOrigin();
  tf::Quaternion inv_quat = inv_transform.getRotation();
  
  printf("Transform: (%f, %f, %f)   [%f, %f, %f, %f]\n", 
	 vehicle_position.x(), vehicle_position.y(), vehicle_position.z(),
         quat.w(), quat.x(), quat.y(), quat.z()
  );
  
  printf("Inv. transform: (%f, %f, %f)   [%f, %f, %f, %f]\n", 
	inv_translation.x(), inv_translation.y(), inv_translation.z(),
	inv_quat.w(), inv_quat.x(), inv_quat.y(), inv_quat.z()
  );
  
  tf::Vector3 relative_position = transform.inverse() * target_location;
  
  printf("Relative position: (%f, %f, %f)\n", relative_position.x(), relative_position.y(), relative_position.z());
}