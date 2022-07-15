#include "ros/ros.h"
#include "ros/console.h"
#include <ros/callback_queue.h>
#include "ros/subscribe_options.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

#include "custom_msgs/mapUpdate.h"
#include "custom_msgs/goalUpdate.h"
#include "custom_msgs/pointData.h"
#include "custom_msgs/pointDataArray.h"

#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <tf_conversions/tf_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>
#include <Eigen/Geometry>
#include <mutex>
#include <vector>
#include <math.h>

// --------------------
// -----Structs-----
// --------------------

struct neighbourMap{
    std::string neighbourName;
    Eigen::Matrix4d transform;
};

struct mapData
{
    std::mutex* mapDataMutex = nullptr;
    std::string robotName;
    Eigen::Vector3d initPosition;
    Eigen::Vector3d initRotation;
    Eigen::Vector3d pose;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
};

struct mapDataConnected
{
    std::string robotName;
    Eigen::Vector3d pose;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Matrix4d transform;
    std::vector<octomap::point3d> nearGoals;
    octomap::point3d goal;
    bool completion = false;
};

// --------------------
// -----Parameters-----
// --------------------

// Map creation parameters
std::string rootRobot;
double resolution;
double percentage;
double minScanRange;

// mapping parameters
double x_positive;
double x_negative;
double y_positive;
double y_negative;
double z_positive;

int blockCount;
int msgCount = 0;
int pclCount = 0;

// Data containers
octomap::point3d origin (0,0,0);

std::vector<mapData> mapDetails;
std::vector<mapDataConnected> mapConnected;
std::vector<octomap::point3d> goals;
std::vector<octomap::point3d> ignorePoints;

pcl::PointCloud<pcl::PointXYZRGB> pointCloud, clusterCloud, centroidCloud;

// components
ros::Publisher pub_goal, pub_map_point, pub_octo, centerArray_pub;
ros::Subscriber maps_sub, goal_sub;

void goalRemoveCallback(const custom_msgs::goalUpdate::ConstPtr &msg){
    Eigen::Matrix<double, 4, 1> robotFrame, serverFrame;

    robotFrame(0,0) = msg->newGoals[0].x;
    robotFrame(1,0) = msg->newGoals[0].y;
    robotFrame(2,0) = msg->newGoals[0].z;
    robotFrame(3,0) = 1;

    for (int i = 0; i < mapConnected.size() ; i++) if (mapConnected[i].robotName == msg->robotNames[0]) serverFrame = mapConnected[i].transform * robotFrame;
    
    octomap::point3d point(serverFrame (0,0), serverFrame (1,0), serverFrame (2,0));

    std::vector<octomap::point3d>::iterator position = std::find(ignorePoints.begin(), ignorePoints.end(), point);
    
    if (position == ignorePoints.end()) ignorePoints.push_back(point);

    ROS_INFO_STREAM("Goal " << serverFrame (0,0) << " " << serverFrame (1,0) << " " << serverFrame (2,0) << " added to unreachable list");
}

void mapCallback(const custom_msgs::mapUpdate::ConstPtr& msg)
{
    double model_resolution;
    bool found = false;
    std::string name;
    Eigen::Vector3d pose, initPosition, initRotation;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGB>());

    pose(0,0) = msg->position.x;
    pose(1,0) = msg->position.y;
    pose(2,0) = msg->position.z;

    pcl::fromROSMsg(msg->pointcloud, *inputCloud);

    name = msg->robotName;
    
    initPosition(0,0) = msg->initialPosition.x;
    initPosition(1,0) = msg->initialPosition.y;
    initPosition(2,0) = msg->initialPosition.z;

    initRotation(0,0) = msg->initialRotation.x;
    initRotation(1,0) = msg->initialRotation.y;
    initRotation(2,0) = msg->initialRotation.z;

    for (int i = 0; i < mapDetails.size() ; i++)
    {
        if (mapDetails[i].robotName == name)
        {
            std::unique_lock<std::mutex> lock(*mapDetails[i].mapDataMutex);
            
            mapDetails[i].pose = pose;
            mapDetails[i].pointCloud = inputCloud;
            mapDetails[i].initPosition = initPosition;
            mapDetails[i].initRotation = initRotation;
            lock.unlock();
            found = true;
        }
    }
    if (!found)
    {
        mapData newData;
        newData.robotName = name;
        newData.mapDataMutex = new std::mutex();
        newData.pose = pose;
        newData.pointCloud = inputCloud;
        newData.initPosition = initPosition;
        newData.initRotation = initRotation;
        mapDetails.push_back(newData);
    }

    ROS_DEBUG_STREAM( "---- " << name << " inputCloud : " <<  inputCloud->size());
}

bool updatePath(std::string nodeName)
{
    mapConnected.clear();
    
    bool rootAvailable = false;
    Eigen::Vector3d rootPosition, rootRotation;

    for (int i = 0; i < mapDetails.size() ; i++)
    {
        if (mapDetails[i].robotName == nodeName)
        {
            rootPosition = mapDetails[i].initPosition;
            rootRotation = mapDetails[i].initRotation;
            rootAvailable = true;
        }
    }

    for (int i = 0; i < mapDetails.size() ; i++)
    {
        Eigen::Affine3f transform = pcl::getTransformation( mapDetails[i].initPosition(0,0)-rootPosition(0,0),
                                                            mapDetails[i].initPosition(1,0)-rootPosition(1,0),
                                                            mapDetails[i].initPosition(2,0)-rootPosition(2,0),
                                                            mapDetails[i].initRotation(0,0)-rootRotation(0,0),
                                                            mapDetails[i].initRotation(1,0)-rootRotation(1,0),
                                                            mapDetails[i].initRotation(2,0)-rootRotation(2,0));

        mapDataConnected newConnectedNeighbour;

        newConnectedNeighbour.robotName  = mapDetails[i].robotName;
        newConnectedNeighbour.pose       = mapDetails[i].pose;
        newConnectedNeighbour.pointCloud = mapDetails[i].pointCloud;
        newConnectedNeighbour.transform  = transform.matrix().cast<double>();

        mapConnected.push_back(newConnectedNeighbour);
    }

    return rootAvailable;
}

void updateMap()
{
    pointCloud.clear();
    
    for (int i = 0; i < mapConnected.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB> tmpPointCloud;

        Eigen::Matrix<double, 4, 1> oldPose, newPose;
        
        oldPose(0,0) = mapConnected[i].pose(0,0);
        oldPose(1,0) = mapConnected[i].pose(1,0);
        oldPose(2,0) = mapConnected[i].pose(2,0);
        oldPose(3,0) = 1;

        newPose = mapConnected[i].transform * oldPose;

        mapConnected[i].pose(0,0) = newPose (0,0);
        mapConnected[i].pose(1,0) = newPose (1,0);
        mapConnected[i].pose(2,0) = newPose (2,0);

        pcl::transformPointCloud(*mapConnected[i].pointCloud, tmpPointCloud, mapConnected[i].transform);

        pointCloud += tmpPointCloud;
    }
    
    ROS_INFO_STREAM("");

    sensor_msgs::PointCloud2 cloudMsg;

    pcl::toROSMsg(pointCloud, cloudMsg);

    cloudMsg.header.frame_id = "map";
    cloudMsg.header.seq = pclCount;

    pub_map_point.publish(cloudMsg);

    pclCount += 1;
}

void updateGoals()
{
    octomap::OcTree map (resolution);
    octomap::Pointcloud bridgeCloud;

    for (int i = 0; i < pointCloud.points.size(); i++)
    {
        octomap::point3d endpoint (pointCloud.points[i].x,pointCloud.points[i].y, pointCloud.points[i].z);
		bridgeCloud.push_back(endpoint);
    }

    map.insertPointCloud(bridgeCloud, origin, -1, true, true);
    map.updateInnerOccupancy(); 	

    goals.clear();

    int x_count_p = (int) (x_positive/(blockCount*resolution));
    int y_count_p = (int) (y_positive/(blockCount*resolution));
    int z_count_p = (int) (z_positive/(blockCount*resolution));

    int x_count_n = (int) (-1*x_negative/(blockCount*resolution));
    int y_count_n = (int) (-1*y_negative/(blockCount*resolution));

    std::vector<octomap::point3d> unknownPointsArray, tempUnknownPoints;
                
    for (int x=x_count_n; x<x_count_p; x++){
        for (int y=y_count_n; y<y_count_p; y++){
            for (int z=0; z<z_count_p; z++){

                int counter = 0;
                octomap::point3d point = octomap::point3d ((blockCount*resolution)*(x+0.5), (blockCount*resolution)*(y+0.5), (blockCount*resolution)*(z+0.5));
            
                double x_min = point.x()-((blockCount-1)*0.5*resolution);
                double x_max = point.x()+(blockCount*0.5*resolution);
                double y_min = point.y()-((blockCount-1)*0.5*resolution);
                double y_max = point.y()+(blockCount*0.5*resolution);
                double z_min = point.z()-((blockCount-1)*0.5*resolution);
                double z_max = point.z()+(blockCount*0.5*resolution);

                for (double a=x_min; a<x_max; a+=resolution){
                    for (double b=y_min; b<y_max; b+=resolution){
                        for (double c=z_min; c<z_max; c+=resolution){
                            if (!map.search(a, b, c)){
                                counter++;                       // number of unidentified voxels
                            }
                        }
                    }
                }

                if (counter>=(percentage*blockCount*blockCount*blockCount)){
                    unknownPointsArray.push_back(point);
                }
            }
        }
    }

    for(int i=0; i<unknownPointsArray.size(); i++){
        bool notfound = true;
        for(int j=0; j<ignorePoints.size(); j++) if (unknownPointsArray[i].distance(ignorePoints[j])<(resolution*2)) notfound = false;
        if (notfound) tempUnknownPoints.push_back(unknownPointsArray[i]);    
    }

    for(int j=0; j<tempUnknownPoints.size(); j++) if (tempUnknownPoints[j].z()<(blockCount*resolution)) goals.push_back(tempUnknownPoints[j]);

    custom_msgs::pointDataArray valueArray;

	for(int i=0; i<goals.size(); i++){
		custom_msgs::pointData msgInstance;

		msgInstance.x = goals[i].x();
		msgInstance.y = goals[i].y();
		msgInstance.z = goals[i].z();

		valueArray.centerPointsArray.push_back(msgInstance);
	}

	centerArray_pub.publish(valueArray);

    octomap_msgs::Octomap octoMsg;
    octomap_msgs::fullMapToMsg(map, octoMsg);

    octoMsg.header.frame_id = "map";
    octoMsg.header.seq = msgCount;  msgCount+=1;
    pub_octo.publish(octoMsg);
}

void selectGoals()
{
    // Cluseter the goals into seperate areas depending on the position of the robot

    for (int i = 0; i < goals.size(); i++)
    {
        octomap::point3d robotPosition (mapConnected[0].pose(0,0), mapConnected[0].pose(1,0), mapConnected[0].pose(2,0));
        std::string robotName = mapConnected[0].robotName;
        double distance = goals[i].distanceXY(robotPosition);

        for (int j = 1; j < mapConnected.size(); j++)
        {
            octomap::point3d newRobotPosition (mapConnected[j].pose(0,0), mapConnected[j].pose(1,0), mapConnected[j].pose(2,0));
            double newDistance = goals[i].distanceXY(newRobotPosition);

            if (newDistance<distance)
            {
                robotName = mapConnected[j].robotName;
                distance = newDistance;
            }
        }

        for (int k = 0; k < mapConnected.size(); k++)
        {
            if (mapConnected[k].robotName==robotName)
            {
                mapConnected[k].nearGoals.push_back(goals[i]);
            }
        }
    }

    //select the most suitable goal out of all the goals. Current approach, closest point is selected

    for (int i = 0; i < mapConnected.size(); i++)
    {
        octomap::point3d robotPosition (mapConnected[i].pose(0,0), mapConnected[i].pose(1,0), mapConnected[i].pose(2,0));
        std::string robotName = mapConnected[i].robotName;

        if (mapConnected[i].nearGoals.size()>0)
        {
            double distance = mapConnected[i].nearGoals[0].distanceXY(robotPosition);
            mapConnected[i].goal = mapConnected[i].nearGoals[0];

            for (int j = 1; j < mapConnected[i].nearGoals.size(); j++)
            {
                double newDistance = mapConnected[i].nearGoals[j].distanceXY(robotPosition);

                if (newDistance<distance)
                {
                    mapConnected[i].goal = mapConnected[i].nearGoals[j];
                }
            }

            mapConnected[i].completion = false;
        }
        else
        {
            mapConnected[i].completion = true;
        }
    }

    custom_msgs::goalUpdate goalMsg;

    for (int i = 0; i < mapConnected.size(); i++)
    {
        custom_msgs::pointData  goal;

        Eigen::Matrix<double, 4, 1> oldGoal, newGoal;

        oldGoal(0,0) = mapConnected[i].goal.x();
        oldGoal(1,0) = mapConnected[i].goal.y();
        oldGoal(2,0) = mapConnected[i].goal.z();
        oldGoal(3,0) = 1;

        newGoal = mapConnected[i].transform.inverse() * oldGoal;

        goal.x = newGoal (0,0);
        goal.y = newGoal (1,0);
        goal.z = newGoal (2,0);

        goalMsg.newGoals.push_back(goal);
        goalMsg.robotNames.push_back(mapConnected[i].robotName);
        goalMsg.completion.push_back(mapConnected[i].completion);
        
        ROS_INFO_STREAM( "Server Frame : " << mapConnected[i].robotName << " : " << mapConnected[i].goal.x() << " " << mapConnected[i].goal.y() << " " << mapConnected[i].goal.z());
        ROS_INFO_STREAM( "Robot Frame  : " << mapConnected[i].robotName << " : " << goal.x << " " << goal.y << " " << goal.z);
    }

    pub_goal.publish(goalMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server_node");
    ros::NodeHandle node;

    node.param<std::string>("map/rootRobot",            rootRobot,                  "rosbot1");

    node.param("map/resolution",                        resolution,                  0.05);
    node.param("map/unexploredPercentage",              percentage,                  0.95);
    node.param("map/sideBlockCount",                    blockCount,                  5);
    node.param("map/minScanRange",                      minScanRange,                0.70);
    node.param("area/front",                            x_positive,                  10.00);
    node.param("area/back",                             x_negative,                  0.00);
    node.param("area/left",                             y_positive,                  10.00);
    node.param("area/right",                            y_negative,                  0.00);
    node.param("area/height",                           z_positive,                  0.75);
    
    ROS_INFO("server_node : loaded parameters");

    ros::SubscribeOptions ops;
    ops.init<custom_msgs::mapUpdate> ("/MapReceive", 10, mapCallback);
    ops.allow_concurrent_callbacks = true;

    maps_sub = node.subscribe(ops);
    goal_sub = node.subscribe("/RejectedGoals", 1, goalRemoveCallback);

    ROS_INFO("server_node : created subscribers");

    pub_goal = node.advertise<custom_msgs::goalUpdate>("/GoalBroadcast", 1, true);
    pub_map_point = node.advertise<sensor_msgs::PointCloud2>("/server_map", 1, true);
    pub_octo = node.advertise<octomap_msgs::Octomap>("/server_octomap", 1, true);

    ROS_INFO("server_node : created service");

    centerArray_pub = node.advertise<custom_msgs::pointDataArray>("goalCenterArray", 1, true);

    ROS_INFO("server_node : created publisher");

    ros::AsyncSpinner spinner (6);
    spinner.start();

    while (ros::ok())
    {
        if (mapDetails.size()>0)
        {
            if(updatePath(rootRobot)) ROS_INFO_STREAM("server_node : updated path"); else continue;

            updateMap();
            ROS_INFO_STREAM("server_node : updated map");

            updateGoals();
            ROS_INFO_STREAM("server_node : updated goals");

            selectGoals();
            ROS_INFO_STREAM("server_node : selected goals");
        }
    }

    return 0;
}
