#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace relation_viz {
    class RelationViz {
        private:
            // node handle
            ros::NodeHandle nHandle;
            // publisher
            ros::Publisher pub = 
                nodeHandle.advertise<visualization_msgs::Marker>(
                    "visualization_marker", 1);
            // subscriber
            ros::Subscriber sub;
            void segmentedObjectsReceived(
                const perception_msgs::SegmentedObjectList::ConstPtr &msg);
        public:
            // constructor
            RelationViz();
    }

    RelationViz::RelationViz(): nHandle("") {
        sub = nHandle.subscribe(
            "segmented_objects", 1000,
            &RelationViz::segmentedObjectsReceived, this);
        pub = nHandle.advertize<sensor_msgs/PointCloud2>(
            "visualized_segmented_objects", 10);
        ROS_INFO("relation_viz node ready")
    }

    RelationViz::segmentedObjsCallback(
    const perception_msgs::SegmentedObjectList::ConstPtr &msg) {
        ROS_INFO("received segmented object list message");

        vector<perception_msgs::SegmentedObject> segmentedMsgs
            = msg.get()->segmentedObjects;

        for (int i = 0; i< segmentedMsgs.size(); i++) {
            //get segmentedObject from message
            perception_msgs::SegmentedObject
                msg = segmentedMsgs[i];
            pcl::PointCloud<pcl::PointXYZ>::Ptr 
                segmentedObjectPCLPointCloudXYZ(
                    new pcl::PointCloud<pcl::PointXYZ>());
            sensor_msgs::PointCloud2 
                msgPointCloud = msg.segmentedObjectPointCloud;
            pcl::PCLPointCloud2
                segmentedObjectPCLPointCloud;
            pcl_conversions::toPCL(
                msgPointCloud,
                segmentedObjectPCLPointCloud);
            pcl::fromPCLPointCloud2(
                    segmentedObjectPCLPointCloud, 
                    *segmentedObjectPCLPointCloudXYZ);
            SegmentedObject segmented = SegmentedObject(
                    msg.segmentedObjectID, 
                    segmentedObjectPCLPointCloudXYZ);
            // TODO uniquely color objects
            // ...

            //build property message from property
            perception_msgs::ObjectCenterProperty objectCenterPropertyMessage;
            objectCenterPropertyMessage.objectCenter = centerOfMassProperty->centerOfMassPoint;
            objectCenterPropertyMessage.segmentedObjectId = centerOfMassProperty->segmentedObjectId;
            objectCenterPropertyMessage.propertyId = centerOfMassProperty->propertyId;

            //send message
            detectedPropertyPublisher.publish(objectCenterPropertyMessage);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "relation_viz");
    // TODO do we need this nHandle? the node?
    ros::NodeHandle nHandle;
    relation_viz::RelationViz node;
    ros::spin();
    return 0;

    /* TODO remove
  	while(nHandle.ok()) {
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
    	vis_pub.publish(marker);
    }
    */
}
