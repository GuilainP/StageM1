#pragma once

/* CONTROL FUNCTIONS FOR EPUCK ROBOT*/

#include <arpa/inet.h>
#include <cstdio> // Pour les Sprintf
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>
#include <unistd.h>
#include <signal.h>


#define INVALID_SOCKET -1
#define SOCKET_ERROR -1

#include "datafile.hpp"
#include "Robot.hpp"

/**** Derive robot pose from encoders and save it along with robot linear and angular velocities ****/
Pose GetCurrPoseFromEncoders(RobotParameters rp, const Pose & prevRobPose, const int& EncL, const int& EncR, const int& prevEncL, const int& prevEncR, Logger& log);

/**** Derive robot pose from vision (from barycenter of target) and save it ****/
Pose GetCurrPoseFromVision(const cv::Point & baryctr, const float & theta, const float & areaPix, Logger& log);


/**** Load image from the disk ****/ //////////// NOT USED IN EPUCK-APP ////////////////////// 
//cv::Mat LoadAndShowImageFromDisk(const std::string folderName, const int& cnt_it);

/**** Load encoder measurements from logs on disk ****/ //////////// NOT USED IN EPUCK-APP ////////////////////// 
// std::vector<int> LoadSensorLogs(const std::string folderName)


/**** Convert IR distances to points x y in robot frame, estimate line parameters y = mx +p in robot frame and then convert everything to world frame ****/
void ConvertIRPointsForWallFollowing(RobotParameters rp, const float dist[10], const Pose & rPoseEnc, cv::Point2f ProxInWFr[10], float & mRob, float & pRob, float & mWorld, float & pWorld);

/**** Draws a map with world frame and robot ****/
void DrawMapWithRobot(RobotParameters rp, const Pose & rPoseEnc, const Pose & rPoseVis, const cv::Point2f ProxInWFr[10], const float mWall, const float pWall);

/**** Process image compute and draw barycenter ****/
// cv::Point ProcessImageToGetBarycenter(cv::Mat& colRawImg, float & areaPix);

/**** Send commands (speed_L, speed_R) to motors in order to realize robot
 * velocities (v, w) for 10 seconds ****/
void SetRobotVelocities(RobotParameters rp, struct timeval startTime, const float& vel, const float& omega, char MotorCmd[15]);

/**** Send commands (speed_L, speed_R) to motors in order to realize robot
 * velocities (v, w) for 10 seconds ****/
void SetVelocities(struct timeval startTime, const float& vel, const float& omega, char MotorCmd[15]);

// control robot using images from the camera
void ControlRobotWithVisualServoing(const cv::Point & baryc, float & vel, float & omega);

// make robot follow a wall using infrared measurements for ten seconds
void ControlRobotToFollowWall(struct timeval startTime, float &vel, float &omega);

//using IR calibration convert the IR values to metric distances
void InfraRedValuesToMetricDistance(RobotParameters rp, const int ProxS[10], float dist[10], Logger& log);

//using the robot parameters, map metric distances to the robot frame
void MetricDistanceToRobFrameCoordinates(RobotParameters rp, const float dist[10], const Pose & rPoseEnc, cv::Point2f ProxInWFr[10], float & mWall, float & pWall);
