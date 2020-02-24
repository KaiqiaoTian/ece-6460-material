// Header file for the class
#include "Homework2.hpp"

// Namespace matches ROS package name
namespace homework2 
{  

 
  // Constructor with global and private node handle arguments
  Homework2::Homework2(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    srv_.setCallback(boost::bind(&Homework2::reconfig, this, _1, _2));

    sub_waypoints_=n.subscribe("/final_waypoints", 1, &Homework2::recvWaypoints, this);
    sub_fix_=n.subscribe("/vehicle/perfect_gps/enhanced_fix", 1, &Homework2::recvFix, this);
    pub_speed_data_ = n.advertise<dataspeed_ulc_msgs::UlcCmd>("/vehicle/ulc_cmd", 1);
  }

  void Homework2::recvWaypoints(const autoware_msgs::LaneConstPtr& msg)
  {
    // store UTM coordinates of each waypoint
    utm_waypoints = msg->waypoints;
    
    
  }

  void Homework2::recvFix(const gps_common::GPSFixConstPtr& msg)
  {
       //update all local frame positions

    std::vector<autoware_msgs::Waypoint> local_frame_points (1);
    for (size_t i=0; i < utm_waypoints.size(); i++){
    autoware_msgs::Waypoint local_waypoint;
      //Do stuff to local frame conversion   tf/vehicle frame.
    
    // Reference waypoints coordinates in UTM frame
    
    double ref_utm_x = utm_waypoints[i].pose.pose.position.x;
    double ref_utm_y = utm_waypoints[i].pose.pose.position.y;
    double waypoint_v =  utm_waypoints[i].twist.twist.linear.x;

    //ROS_INFO("lat and lon : (%f, %f)", reference_coordinates.latitude, reference_coordinates.longitude);
    //ROS_INFO("----------------------------------------------------------------");

      // Vehicle Current position
    gps_common::GPSFix current_position;
    current_position.latitude = msg->latitude;
    current_position.longitude = msg->longitude;   //gps data, NED

    double r;
    double a;
    double ENU_heading;
    double UTM_heading;
    double central = -81;
    
    a = msg->track;

    if(90 < a) 
    {
      ENU_heading = 90 - a;    
    }
    else
      ENU_heading = 360 + 90 - a ; 
    r = atan(tan(current_position.longitude - central)*sin(current_position.latitude));
    UTM_heading = ENU_heading +r;

    current_position.track = UTM_heading* M_PI / 180.0;


    // Convert current position to UTM from gps to utm
    double current_utm_x;
    double current_utm_y;
    std::string current_utm_zone;
    gps_common::LLtoUTM(current_position.latitude, current_position.longitude, 
                        current_utm_y, current_utm_x, current_utm_zone);
   // std::cout << "vehicle UTM position:      (" << current_utm_x << ", " << current_utm_y << ") Zone: " << current_utm_zone << "\n";

    // Compute relative position vector in UTM
    double relative_pos_east = current_utm_x - ref_utm_x;
    double relative_pos_north = current_utm_y - ref_utm_y;
   // std::cout << "Our current position is " << relative_pos_east << " meters East and "
    //          << relative_pos_north << " meters North of the reference point\n";


    //double heading = -135.0 * M_PI / 180.0;
    //current_position.track = (msg->track)* M_PI / 180.0;; // This is where the simulated heading will be; in the 'track' field

    tf::Transform utm_to_vehicle;
    utm_to_vehicle.setOrigin(tf::Vector3(current_utm_x, current_utm_y, 0.0));
    utm_to_vehicle.setRotation(tf::createQuaternionFromYaw(current_position.track));

    tf::Vector3 ref_utm_vect(ref_utm_x, ref_utm_y, 0.0);

    tf::Vector3 vehicle_frame_vect = utm_to_vehicle.inverse() * ref_utm_vect;
   // std::cout << "The reference point is " << vehicle_frame_vect.x() << " meters in front and "
   //           << vehicle_frame_vect.y() << " meters to the left of our current heading\n";

    local_waypoint.pose.pose.position.x = vehicle_frame_vect.getX();
    local_waypoint.pose.pose.position.y = vehicle_frame_vect.getY();  
  if (utm_waypoints.size() > 24){    
    local_waypoint.twist.twist.linear.x = waypoint_v; 
    }
  else{
    local_waypoint.twist.twist.linear.x = 0; 
  }

    local_frame_points.push_back(local_waypoint);
  }

       //select a target point based on lookahead dist
    int p = 24; //before 25 is better
    
    
       //Compute turning curvature from target point
    double x = local_frame_points[p].pose.pose.position.x;
    double y = local_frame_points[p].pose.pose.position.y;
    double v = local_frame_points[p].twist.twist.linear.x;
    double R;
    double k;
    //std::cout << "p20 " << x << " x "<< v<< " speed\n";

    //R = (x*x + y*y)/2*y;
    k = 2*y/(x*x + y*y);

       //populate a UlcCmd message with curvature and copy the speed from
       //the slected target point 

    dataspeed_ulc_msgs::UlcCmd pub_msg;

    pub_msg.clear = false;
    pub_msg.enable_pedals = true;
    pub_msg.enable_steering = true;
    pub_msg.linear_velocity = v;
    pub_msg.yaw_command =k;
    pub_msg.steering_mode = 1;
    //ROS_INFO("linear and yaw : (%f, %f)", v, k);
       //publish UlcCmd
    pub_speed_data_.publish(pub_msg);
     }

   void Homework2::reconfig(Homework2Config& config, uint32_t level)
  {
    cfg_ = config;
  }
 

}
