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
            ros::Subscriber segmentedSub;
            std::vector<int32_t> colors;
            void segmentedObjsReceived(
                const perception_msgs::SegmentedObjectList::ConstPtr &msg);
        public:
            // constructor
            RelationViz();
    };

    RelationViz::RelationViz(): nHandle("") {
        segmentedSub = nHandle.subscribe("segmented_objects", 10, &RelationViz::segmentedObjsReceived, this);
        segmentedPub = nHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("visualized_segmented_objects", 10);

        // create vector of colors
        int32_t rgb;
        for (uint8_t r = 0; r <= 4; ++r) {
            for (uint8_t g = 0; g <= 4; ++g) {
                for (uint8_t b = 0; b <= 4; ++b) {
                    rgb = (r*85 << 16) | (g*85 << 8) | b*85;
                    colors.push_back(rgb);
                }
            }
        }

        ROS_INFO("relation_viz node ready");
    }

    void RelationViz::segmentedObjsReceived(const perception_msgs::SegmentedObjectList::ConstPtr &msg) {
        ROS_INFO("received segmented object list message");

        // add all segments to one cloud before publishing
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segments (new pcl::PointCloud<pcl::PointXYZRGB>);

        // iterate through all segmented cloud and uniquely color each
        int32_t rgb;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented (new pcl::PointCloud<pcl::PointXYZRGB>);
        std::vector<perception_msgs::SegmentedObject> segmentedMsgs = msg.get()->segmentedObjects;
        for (int i = 0; i< segmentedMsgs.size(); i++) {

            // get segmented cloud and convert to pcl::PointCloud
            pcl::fromROSMsg(segmentedMsgs[i].segmentedObjectPointCloud, *segmented);

            // add color to each point in segment
            rgb = colors[i % colors.size()];
            for (int j = 0; j < segmented->points.size(); ++j) {
                segmented->points[j].rgb = *reinterpret_cast<float*>(&rgb);
            }

            // add colored segment to cloud being published
            *segments += *segmented;
        }

        // set frame
        segments->header.frame_id = msg.get()->segmentedObjects[0].segmentedObjectPointCloud.header.frame_id;

        // publish
        segmentedPub.publish(segments);
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
