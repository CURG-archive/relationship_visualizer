#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
int main(int argc, char **argv) 
{

  ros::init(argc, argv, "relationship_visualizer");
  ros::NodeHandle node_handle;

  ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );

  	while(node_handle.ok())
  	{
  		ros::Duration(0.5).sleep();
  		ROS_INFO("sending message");
	    visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 1;
		marker.pose.position.y = 1;
		marker.pose.position.z = 1;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
		//only if using a MESH_RESOURCE marker type:
		marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    	vis_pub.publish( marker );
  }
  return 0;
}
