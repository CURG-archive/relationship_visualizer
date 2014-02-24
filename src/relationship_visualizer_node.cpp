#include <ros/ros.h>
#include <ros/package.h>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception_msgs/SegmentedObject.h"
#include "perception_msgs/SegmentedObjectList.h"
#include "perception_msgs/ObjectCenterProperty.h"

namespace relation_viz {
    class RelationViz {
        private:
            ros::NodeHandle nHandle;
            ros::Publisher  pub;
            ros::Subscriber sub;
            void segmentedObjsReceived(
                const perception_msgs::SegmentedObjectList::ConstPtr &msg);
        public:
            // constructor
            RelationViz();
    };

    RelationViz::RelationViz(): nHandle("") {
        sub = nHandle.subscribe("segmented_objects", 1000, &RelationViz::segmentedObjsReceived, this);
        pub = nHandle.advertise<sensor_msgs::PointCloud2>("visualized_segmented_objects", 10);
        ROS_INFO("relation_viz node ready");
    }

    void RelationViz::segmentedObjsReceived(
    const perception_msgs::SegmentedObjectList::ConstPtr &msg) {
        ROS_INFO("received segmented object list message");

        std::vector<perception_msgs::SegmentedObject> segmentedMsgs = msg.get()->segmentedObjects;

        for (int i = 0; i< segmentedMsgs.size(); i++) {
            pub.publish(segmentedMsgs[i].segmentedObjectPointCloud);
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
