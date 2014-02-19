#include <ros/ros.h>
#include "perception_msgs/ObjectCenterProperty.h"
#include "perception_msgs/SegmentedObjectList.h"
#include "perception_msgs/SegmentedObject.h"
#include "gtest/gtest.h"
#include "std_msgs/String.h"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

// TODO why do we use this namespace?
namespace relationship_visualizer_node_test {
    // TODO renamed to TestVizNode
    class TestVizNode {
        private:
            ros::NodeHandle node_handle;
        public:
            TestVizNode();
            // TODO will publish segmented objects for the relationshipVisualizerNode
            ros::Publisher segmentedObjectsPublisher;
            ros::Subscriber detectedPropertiesSubscriber;

            void relatedObjectsMessageCallback(
                const perception_msgs::ObjectCenterProperty::ConstPtr &msg
            );
            bool hasReceivedMessage;
            perception_msgs::ObjectCenterProperty receivedMsg;
    };

    TestVizNode::TestVizNode(): node_handle("") {
        detectedPropertiesSubscriber = node_handle.subscribe(
            "object_properties", 1000, 
            &TestVizNode::relatedObjectsMessageCallback, this
        );
        segmentedObjectsPublisher = 
            node_handle.advertise<perception_msgs::SegmentedObjectList>(
                "segmented_objects",10
        );
        hasReceivedMessage = false;
        ROS_INFO("test_viz_node ready");
    }

    void TestVizNode::relatedObjectsMessageCallback(const perception_msgs::ObjectCenterProperty::ConstPtr &msg) {
        ROS_INFO("Received related object list message");
        hasReceivedMessage = true;
        receivedMsg = *msg;
    }

}

//create a node to send messages to the relationship_visualizer_node so we can validate its responses.
relationship_visualizer_node_test::TestVizNode buildTestNode() {
    relationship_visualizer_node_test::TestVizNode node;
    //give test node time to initialize VERY IMPORTANT
    // TODO is there another way to verify initialization?
    ros::Duration(1).sleep();
    return node;
}

perception_msgs::SegmentedObjectList buildAppleSegmentedObjectsList() {
    // get point cloud
    std::string fileName = ros::package::getPath("object_models") + "/models/rgbd-dataset/apple/apple_1/apple_1_1_100.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // load the file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n")
    }

    sensor_msgs::PointCloud2 sensorMessagePointCloud;
    pcl::toROSMsg(*cloud,sensorMessagePointCloud);

    // build segmented object and add point cloud to it
    perception_msgs::SegmentedObject segmentedObject;
    segmentedObject.segmentedObjectID = 1;
    segmentedObject.segmentedObjectPointCloud = sensorMessagePointCloud;

    // build segmentedObjectList and add segmented object to it.
    perception_msgs::SegmentedObjectList segmentedObjectsList;
    segmentedObjectsList.segmentedObjects.push_back(segmentedObject);
    return segmentedObjectsList;
}


TEST(RELATIONSHIP_VISUALIZER_TEST_NODE, TestEmptySegmentedObjectsList) {
    // create test node
    relationship_visualizer_node_test::TestVizNode node = buildTestNode();
    // create segm-obj list
    perception_msgs::SegmentedObjectList segmentedObjectsList = buildAppleSegmentedObjectsList();
    // publish segm-obj list
    node.segmentedObjectsPublisher.publish(segmentedObjectsList);
    // spin until msgs received
    double start = ros::Time::now().toSec();
    while (!node.hasReceivedMessage) {
      ros::spinOnce();
      // if no msgs received for 5 secs, something is wrong.
      double now = ros::Time::now().toSec();
      if (now - start > 5.0) {
          break;
      }
    }
    EXPECT_EQ(node.hasReceivedMessage, true);

    double absErrorBound = .0001;
    ASSERT_NEAR(node.receivedMsg.objectCenter.x, -0.0127071, absErrorBound);
    ASSERT_NEAR(node.receivedMsg.objectCenter.y, 0.699493, absErrorBound);
    ASSERT_NEAR(node.receivedMsg.objectCenter.z, -0.0152639, absErrorBound);
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "test_relationship_visualizer_node");
  ros::NodeHandle nh;

  //give the relationship_visualizer_node time to start up.
  ros::Duration(1).sleep();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

