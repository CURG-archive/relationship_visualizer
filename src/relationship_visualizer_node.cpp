#include <ros/ros.h>
#include <ros/package.h>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception_msgs/SegmentedObject.h"
#include "perception_msgs/SegmentedObjectList.h"
#include "perception_msgs/ObjectCenterProperty.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"

namespace relation_viz {
    class RelationViz {
        private:
            ros::NodeHandle nHandle;
            ros::Publisher  segmentedPub;
            ros::Subscriber sub;
            void segmentedObjsReceived(
                const perception_msgs::SegmentedObjectList::ConstPtr &msg);
        public:
            // constructor
            RelationViz();
    };

    RelationViz::RelationViz(): nHandle("") {
        sub = nHandle.subscribe("segmented_objects", 1000, &RelationViz::segmentedObjsReceived, this);
        segmentedPub = nHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("visualized_segmented_objects", 10);
        ROS_INFO("relation_viz node ready");
    }

    void RelationViz::segmentedObjsReceived(const perception_msgs::SegmentedObjectList::ConstPtr &msg) {
        ROS_INFO("received segmented object list message");

        std::vector<perception_msgs::SegmentedObject> segmentedMsgs = msg.get()->segmentedObjects;
        uint8_t r = 255, g = 0, b = 0;
        int32_t rgb = (r << 16) | (g << 8) | b;

        for (int i = 0; i< segmentedMsgs.size(); i++) {
            // sensor_msg::PointCloud2 -> pcl::PointCloud<PointXYZRGB>
            pcl::PointCloud<pcl::PointXYZRGB> *segmented = new pcl::PointCloud<pcl::PointXYZRGB>();
            pcl::fromROSMsg(segmentedMsgs[i].segmentedObjectPointCloud, *segmented);
            // color segmented object
            // TODO uniquely color, don't just color all red
            for (int i = 0; i < segmented->points.size(); ++i) {
                segmented->points[i].rgb = *reinterpret_cast<float*>(&rgb);
            }
            segmentedPub.publish(*segmented);
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
}
