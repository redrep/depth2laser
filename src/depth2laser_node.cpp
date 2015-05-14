#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <math.h>

//GLOBALS
//=====================================================================

//Topics and messages
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ros::Subscriber info_sub;
ros::Subscriber frame_sub;
ros::Publisher xtion_pub;
ros::Publisher laser_pub;
ros::Publisher scan_pub;
sensor_msgs::CameraInfo camerainfo;
sensor_msgs::PointCloud cloud;
sensor_msgs::PointCloud laser;
sensor_msgs::LaserScan scan;
//EIGEN
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Eigen::Matrix3f K;
Eigen::Vector3f worldPoint;
Eigen::Vector3f planeCoeffs;
Eigen::Vector3f planePassingPoint;
Eigen::Vector4f plane;
Eigen::Isometry3f xtionTransfom;
Eigen::Isometry3f laserTransfom;
//TF
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
tf::TransformListener* listener;
tf::StampedTransform xtion;
tf::StampedTransform xtion2laser;
//Other
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool gotInfo=false;
//Laser
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define LASER_BUFFER_ELEMS 5000
float laser_buffer[LASER_BUFFER_ELEMS];
//=====================================================================


void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
    //K matrix
    //================================================================================
    camerainfo.K = info->K;
    ROS_INFO("Got camera info!");
    K(0,0) = camerainfo.K.c_array()[0];
    K(0,1) = camerainfo.K.c_array()[1];
    K(0,2) = camerainfo.K.c_array()[2];
    K(1,0) = camerainfo.K.c_array()[3];
    K(1,1) = camerainfo.K.c_array()[4];
    K(1,2) = camerainfo.K.c_array()[5];
    K(2,0) = camerainfo.K.c_array()[6];
    K(2,1) = camerainfo.K.c_array()[7];
    K(2,2) = camerainfo.K.c_array()[8];

    //TF
    //================================================================================
    ROS_INFO("waif for tf...");
    listener->waitForTransform("/world","/xtion",ros::Time(0),ros::Duration(30));
    listener->lookupTransform("/world", "/xtion",ros::Time(0), xtion);
    listener->waitForTransform("/xtion","/laser",ros::Time(0),ros::Duration(30));
    listener->lookupTransform("/xtion", "/laser",ros::Time(0), xtion2laser);
    ROS_INFO("got");
    xtionTransfom.setIdentity();
    xtionTransfom.translation() << xtion.getOrigin().x(),xtion.getOrigin().y(),xtion.getOrigin().z();
    xtionTransfom.translation()*=1000.0f;//mm
    xtionTransfom.translation()<<0,0,0;

    laserTransfom.setIdentity();
    laserTransfom.translation() << xtion2laser.getOrigin().x(),xtion2laser.getOrigin().y(),xtion2laser.getOrigin().z();
    std::cout<<"LASERT "<<laserTransfom.translation().transpose()<<std::endl;
    Eigen::Quaternionf l(xtion2laser.getRotation().getW(),xtion2laser.getRotation().getX(),xtion2laser.getRotation().getY(),xtion2laser.getRotation().getZ());
    laserTransfom.linear()=l.toRotationMatrix();
    //bringin the pointcloud into the XTION frame (not the optical one!)
    Eigen::AngleAxisf yawAngle(0, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle(M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(-M_PI/2, Eigen::Vector3f::UnitX());
    Eigen::Quaternionf q = rollAngle * pitchAngle * yawAngle;
    xtionTransfom.linear() = q.toRotationMatrix();
    //PLANE STUFF
    //================================================================================
    planeCoeffs<<0.0f,0.0f,1.0f;
    Eigen::Isometry3f rotator;
    rotator.setIdentity();
    Eigen::Quaternionf t(xtion2laser.getRotation().getW(),
                         xtion2laser.getRotation().getX(),
                         xtion2laser.getRotation().getY(),
                         xtion2laser.getRotation().getZ());
    rotator.linear() = t.toRotationMatrix();
    //transforming the Z unit vector using the frame transform.
    rotator.setIdentity();
    planeCoeffs=rotator*planeCoeffs;
    std::cout<<planeCoeffs.transpose()<<std::endl;
    planePassingPoint.setZero();
    //planePassingPoint<<xtion2laser.getOrigin().x(),xtion2laser.getOrigin().y(),xtion2laser.getOrigin().z();
    std::cout<<"POINT "<<planePassingPoint<<std::endl;
    float d = -(planePassingPoint(0)*planeCoeffs(0)+
                planePassingPoint(1)*planeCoeffs(1)+
                planePassingPoint(2)*planeCoeffs(2));
    plane<<planeCoeffs(0),planeCoeffs(1),planeCoeffs(2),d;

    //Don't want to receive any camera info stuff, basically this callback is used a init function
    info_sub.shutdown();
    //set flag. Bye.
    gotInfo=true;
}

void frameCallback(const sensor_msgs::Image::ConstPtr& frame)
{
    if(!gotInfo) return;
    //convert the image message to a standard opencv Mat.
    //NOTE: data is immutable, it's ok with that.
    cloud.points.clear();
    laser.points.clear();
    scan.ranges.clear();
    int scan_points=0;
    float minx=0;
    float miny=0;

    float maxx=0;
    float maxy=0;
    geometry_msgs::Point32 point;
    const ushort* row_ptr;
    cv_bridge::CvImageConstPtr image =  cv_bridge::toCvShare(frame);
    for(int i =0;i<image->image.rows;i++){
        row_ptr = image->image.ptr<ushort>(i);
        for(int j=0;j<image->image.cols;j++){
            if(row_ptr[j]!=0){
                worldPoint<<j*row_ptr[j],i*row_ptr[j],row_ptr[j];
                worldPoint = K.inverse()*worldPoint;
                worldPoint= xtionTransfom*worldPoint;
                worldPoint/=1000.0f;
                point.x=worldPoint[0];
                point.y=worldPoint[1];
                point.z=worldPoint[2];
                cloud.points.push_back(point);
                //checking if the point lies on the requested plane.
                //if so, i'll add it to the laser pointcloud
                worldPoint=laserTransfom.inverse()*worldPoint;
                point.x=worldPoint[0];
                point.y=worldPoint[1];
                point.z=worldPoint[2];
                //laser.points.push_back(point);
                //if((worldPoint(0)*plane(0)+worldPoint(1)*plane(1)+worldPoint(2)*plane(2)+plane(3))<=0.01f &&
                //(worldPoint(0)*plane(0)+worldPoint(1)*plane(1)+worldPoint(2)*plane(2)+plane(3))>=-0.01f){
                if(worldPoint[2]<=0.005f && worldPoint[2]>=-0.005f){
                    scan_points++;
                    point.x=worldPoint[0];
                    point.y=worldPoint[1];
                    point.z=worldPoint[2];
                    if(worldPoint[1]<miny){
                        miny=worldPoint[1];
                        minx=worldPoint[0];
                    }
                    if(worldPoint[1]>maxy){
                        maxy=worldPoint[1];
                        maxx=worldPoint[0];
                    }
                    //scan.intensities.push_back(sqrt(pow(minx=worldPoint[0],2)+pow(minx=worldPoint[1],2)));
                    laser.points.push_back(point);
                }
            }
        }

    }
    //scan.angle_min=atan(miny/minx);
    //scan.angle_max=atan(maxy/maxx);
    scan.header.stamp=ros::Time::now();
    scan.angle_min=-M_PI;
    scan.angle_max=M_PI;
    scan.range_min=0;
    scan.range_max=20.0f;
    scan.angle_increment=2*M_PI/LASER_BUFFER_ELEMS;
    //laser
    //memset(laser_buffer,0,4*LASER_BUFFER_ELEMS);
    for(int i = 0;i<LASER_BUFFER_ELEMS;i++){
        laser_buffer[i]=INFINITY;
    }
    for(int i=0;i<laser.points.size();i++){
        float x = laser.points.at(i).x;
        float y = laser.points.at(i).y;
        float ro   = sqrt(pow(x,2)+pow(y,2));
        //std::cout<<y<<" "<<x<<std::endl;
        float teta = atan(y/x);
        int angle_bin = (teta*LASER_BUFFER_ELEMS/(2*M_PI))+(LASER_BUFFER_ELEMS/2);
        laser_buffer[angle_bin]=ro;

    }
    for(int i=0;i<LASER_BUFFER_ELEMS;i++){
        scan.ranges.push_back(laser_buffer[i]);
    }
//    std::cout<< "MINY "<<miny<<"\tMAXY "<<maxy<<std::endl;
//    std::cout<< "MINX "<<minx<<"\tMAXX "<<maxx<<std::endl;
//    std::cout<< "MIN "<<scan.angle_min<<"\tMAX "<<scan.angle_max<<std::endl<<std::endl;
    xtion_pub.publish(cloud);
    laser_pub.publish(laser);
    scan_pub.publish(scan);
}


//================================================================================
//MAIN PROGRAM
//================================================================================
int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    listener= new tf::TransformListener();
    //Messages headers for TF
    //================================================================================
    cloud.header.frame_id="xtion";
    laser.header.frame_id="laser";
    scan.header.frame_id="laser";
    ros::NodeHandle n;
    //Subscribers
    //================================================================================
    frame_sub = n.subscribe("/camera/depth/image_raw", 1, frameCallback);
    info_sub = n.subscribe("/camera/depth/camera_info", 1, infoCallback);
    //Publishers
    //================================================================================
    xtion_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 1);
    laser_pub = n.advertise<sensor_msgs::PointCloud>("laser", 1);
    scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1);
    ros::spin();

    return 0;
}
