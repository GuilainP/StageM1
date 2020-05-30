#include <controlFunctions.hpp>

/**** Send commands (speed_L, speed_R) to motors in order to realize robot
 * velocities (v, w) for 10 seconds ****/
void SetVelocities(struct timeval startTime, const float& vel, const float& omega, char MotorCmd[15]) {
    std::cout << "The required robot velocities are v "<< vel <<" w "<< omega <<"\n\n\n";
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    long int timeSinceStart =
            ((curTime.tv_sec * 1000000 + curTime.tv_usec) -
             (startTime.tv_sec * 1000000 + startTime.tv_usec)) /
            1000;
    std::cout << "timeSinceStart = " << timeSinceStart << " ms\n";
    // Commands to be sent to the left and right wheels
    int speedG;
    int speedD;
    if (timeSinceStart < 10000) {
        speedG = 0;
        speedD = 0;
        // Sends the command to the epuck motors
        sprintf(MotorCmd, "D,%d,%d", speedG, speedD);
    } else {
        sprintf(MotorCmd, "D,%d,%d", 0, 0);
    }
    std::cout << "SetRobotVelocities wheel speeds are "<< speedG << " " << speedD;
}
/**** Convert IR distances to points x y in robot frame, estimate line parameters y = mx +p in robot frame and then convert everything to world frame ****/
void ConvertIRPointsForWallFollowing(RobotParameters rp, const float dist[10], const Pose & rPoseEnc, cv::Point2f ProxInWFr[10], float & mRob, float & pRob, float & mWorld, float & pWorld) {
    cv::Point2f ProxInRobFrame[10];
    for (int i = 0; i < 10; i++) {
        std::cout << "the angle of IR number " << i << " is " << rp.theta[i] <<" and the distance is " << dist[i] << std::endl;
        ProxInRobFrame[i].x = 0;
        ProxInRobFrame[i].y = 0;
    }
    mRob = 0;
    pRob = 0;
    std::cout << "The robot's current pose is X " << rPoseEnc.x << " Y " << rPoseEnc.y << " Theta " << rPoseEnc.th << std::endl;
    for (int i = 0; i < 10; i++) {
        ProxInWFr[i].x = 0;
        ProxInWFr[i].y = 0;
    }
    mWorld = -.5;
    pWorld = 0.05;
}
/**** Derive robot pose from encoders and save it along with robot linear and angular velocities ****/
Pose GetCurrPoseFromEncoders(RobotParameters rp, const Pose & prevRobPose, const int& EncL, const int& EncR, const int& prevEncL, const int& prevEncR, Logger& log){
    std::cout << "Left Encoder : "<< EncL <<" ";
    std::cout << "Right Encoder : "<< EncR <<" \n";
    const float encoderFactor = 0.0072;//with bias
    const float sampleTime = 0.1;//TODO this should also be read from logs

    float linVel, angVel, dx, dy, dTh;
    float dEncL = EncL - prevEncL;
    float dEncR = EncR - prevEncR;
    if ((fabs(dEncL) > 150) || (fabs(dEncR) > 150)) {//fix measuring errors
        linVel = 0;
        angVel = 0;
    } else {
        linVel = rp.wheel_radius * encoderFactor * (dEncR + dEncL) / 2;
        angVel = rp.wheel_radius * encoderFactor * (dEncR - dEncL) / rp.axis_length;
    }
    dTh = angVel * sampleTime;
    dx = linVel * cos(prevRobPose.th +dTh);
    dy = linVel * sin(prevRobPose.th +dTh);
    Pose curRobPose(prevRobPose.x+dx, prevRobPose.y+dy, prevRobPose.th +dTh);

    //log the data

    log.addIn(log.file_v, linVel);
    log.addIn(log.file_w, angVel);
    log.addIn(log.file_dx, dx/sampleTime);
    log.addIn(log.file_dy, dy/sampleTime);
    log.addIn(log.file_x, curRobPose.x);
    log.addIn(log.file_y, curRobPose.y);
    log.addIn(log.file_th, curRobPose.th);
    log.addIn(log.file_time, sampleTime); //TODO this should also be read from logs

    std::cout << "curRobPose from encoders is x " << curRobPose.x << " y " << curRobPose.y << std::endl;
    return(curRobPose);
}

/**** Derive robot pose from vision (from barycenter of target) and save it ****/
Pose GetCurrPoseFromVision(const cv::Point & baryctr, const float & theta, const float & areaPix, Logger& log){
    Pose curRobPose;
    curRobPose.setPose(.32, 0., M_PI);//TODOM2

    //log the data

    log.addIn(log.file_x_vision, curRobPose.x);
    log.addIn(log.file_y_vision, curRobPose.y);
    log.addIn(log.file_th_vision, curRobPose.th);

    std::cout << "curRobPose from vision is x " << curRobPose.x << " y " << curRobPose.y << std::endl;
    return(curRobPose);
}
/**** Show image from the robot camera and save it on the disk ****/ 
/*
cv::Mat LoadAndShowImageFromDisk(const std::string folderName, const int& cnt_it) {
    //load the image from the disk
    char imgNameIteration[30];
    sprintf(imgNameIteration, "images/image%04d.jpg", cnt_it);
    cv::Mat colorRawImg = cv::imread(folderName + imgNameIteration);
    std::cout << "\nLoadImage loaded image " << folderName + imgNameIteration << std::endl;
    cv::namedWindow( "loggedImg", cv::WINDOW_AUTOSIZE ); // Create a window for display
    cv::moveWindow( "loggedImg", 0,0); // Move window
    cv::imshow( "loggedImg", colorRawImg);                // Show the image inside it
    return(colorRawImg);
} 
*/

/**** Load encoder measurements from logs on disk ****/ //////////// NOT USED IN EPUCK-APP //////////////////////
/*
std::vector<int> LoadSensorLogs(const std::string folderName) {
    std::vector<int> sensData;
    //load the data from the disk
    FILE *sen;
    sen = fopen(folderName.c_str(), "r");
    std::cout << "Loading sensor from " << folderName << std::endl;
    int i=0;
    int tmp;
    while(fscanf(sen,"%d",&tmp)!=EOF) {
        sensData.push_back(tmp);
        std::cout << "i " << i <<"\t sen "<< sensData[i] <<"\n";
        i++;
    };
    int size = i;
    std::cout << "size " << size << std::endl;
    return(sensData);
}
*/

void DrawMapWithRobot(RobotParameters rp, const Pose & rPoseEnc, const Pose & rPoseVis, const cv::Point2f ProxInWorldFrame[10], const float mWall, const float pWall){
    cv::Mat mapImg;
    static const int cmInPixels = 15;
    static const int xMaxInCm = 60;
    static const int yMaxInCm = 40;
    static const int xOriginInCm = 5;
    //draw empty map
    mapImg.create((xMaxInCm + xOriginInCm)*cmInPixels, yMaxInCm*cmInPixels, CV_8UC3);
    mapImg.setTo(cv::Scalar(255,255,255));

    cv::Point origin((yMaxInCm/2)*cmInPixels,xOriginInCm*cmInPixels);
    //draw grid
    //draw horizontal
    for (int i = 0; i <= 32; i++) {
        cv::line(mapImg, origin + cv::Point(-11*cmInPixels, i*cmInPixels), origin + cv::Point(11*cmInPixels, i*cmInPixels), cv::Scalar(200, 200, 200),1);
    }
    //draw vertical
    for (int i = 0; i <= 22; i++) {
        cv::line(mapImg, origin + cv::Point(-11*cmInPixels + i*cmInPixels, 0), origin + cv::Point(-11*cmInPixels + i*cmInPixels, 32*cmInPixels), cv::Scalar(200, 200, 200),1);
    }

    //draw scale
    cv::line(mapImg, cv::Point(xOriginInCm*cmInPixels,xOriginInCm*cmInPixels), cv::Point(xOriginInCm*cmInPixels,(xOriginInCm + 1)*cmInPixels), cv::Scalar(0, 100, 0),2);
    std::string cmText = "1 cm";
    cv::putText(mapImg, cmText, cv::Point(xOriginInCm*cmInPixels+3,(xOriginInCm+0.5)*cmInPixels), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 100, 0),2);

    //draw x axis
    cv::arrowedLine(mapImg, origin, origin + cv::Point(0, xMaxInCm/3*cmInPixels), cv::Scalar(255, 0, 0),2,8,0,0.03);
    std::string xText = "X";
    cv::putText(mapImg, xText, origin + cv::Point(- cmInPixels, cmInPixels), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0),2);

    //draw y axis
    cv::arrowedLine(mapImg, origin, origin + cv::Point(yMaxInCm/3*cmInPixels, 0), cv::Scalar(255, 0, 0),2,8,0,0.03);
    std::string yText = "Y";
    cv::putText(mapImg, yText, origin + cv::Point(cmInPixels, -cmInPixels), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 0, 0),2);

    //draw robot pose from encoders
    float thEnc  = rPoseEnc.th;
    float uEnc = (rPoseEnc.y*100 + yMaxInCm/2)*cmInPixels;
    float vEnc = (rPoseEnc.x*100 + xOriginInCm)*cmInPixels;
    float robRadius = rp.robot_radius*100*cmInPixels;
    cv::line(mapImg, cv::Point(uEnc,vEnc), cv::Point(uEnc-robRadius*sin(thEnc-M_PI),vEnc-robRadius*cos(thEnc-M_PI)), cv::Scalar(0, 0, 255),2);
    cv::circle(mapImg, cv::Point(uEnc,vEnc), robRadius, cv::Scalar(0, 0, 255), 2);

    //draw IR
    for (int i = 0; i < 10; i++) {
        cv::Scalar col = cv::Scalar(0, 0, 0);
        int thickness = 3;
        cv::Point IRinMap;
        float d = sqrt((ProxInWorldFrame[i].x - rPoseEnc.x)*(ProxInWorldFrame[i].x - rPoseEnc.x) +
                       (ProxInWorldFrame[i].y - rPoseEnc.y)*(ProxInWorldFrame[i].y - rPoseEnc.y));
        if (d - rp.robot_radius >= rp.dist_max) {
            col = cv::Scalar(255, 0, 0);
            thickness = 1;
        }
        //std::cout << " ProxInRobFrame " << i << " is " << ProxInRobFrame[i].x << " - " << ProxInRobFrame[i].y << std::endl;
        IRinMap.x = (ProxInWorldFrame[i].y * 100  + yMaxInCm/2) *cmInPixels;
        IRinMap.y = (ProxInWorldFrame[i].x * 100  + xOriginInCm) *cmInPixels;
        //std::cout << " IRinMap " << IRinMap << " rob center " << uEnc << " " << vEnc << std::endl;
        cv::line(mapImg, cv::Point(uEnc,vEnc), IRinMap, col, thickness);
    }
    //draw wall
    cv::line(mapImg, cv::Point((pWall*100 +yMaxInCm/2)*cmInPixels, xOriginInCm*cmInPixels),
         cv::Point(((mWall + pWall)*100 + yMaxInCm/2)*cmInPixels, (100+xOriginInCm)*cmInPixels),
       //cv::Point(xOriginInCm*cmInPixels, (pWall*100 + yMaxInCm/2)*cmInPixels),
         //          cv::Point((100+xOriginInCm)*cmInPixels, ((mWall + pWall)*100 + yMaxInCm/2)*cmInPixels),
                    cv::Scalar(0, 0, 255), 2);

    //draw robot pose from vision
    float xVis = rPoseVis.x;
    float yVis = rPoseVis.y;
    float thVis  = rPoseVis.th;
    float uVis = (yVis*100 + yMaxInCm/2)*cmInPixels;
    float vVis = (xVis*100 + xOriginInCm)*cmInPixels;
    cv::line(mapImg, cv::Point(uVis,vVis), cv::Point(uVis-robRadius*sin(thVis-M_PI),vVis-robRadius*cos(thVis-M_PI)), cv::Scalar(0, 0, 0),1);
    cv::circle(mapImg, cv::Point(uVis,vVis), robRadius, cv::Scalar(0, 0, 0), 1);

    //show map
    cv::namedWindow( "map", cv::WINDOW_AUTOSIZE ); // Create a window for display
    cv::moveWindow( "map", 1020,0); // Move window
    cv::imshow( "map", mapImg);                // Show the image inside it
}

cv::Point ProcessImageToGetBarycenter(cv::Mat& colRawImg, float & areaPix) {
    struct timeval start, end;
    gettimeofday(&start, NULL); //get start time of function
    cv::Mat greyImg;
    cv::cvtColor(colRawImg, greyImg, cv::COLOR_BGR2GRAY); //convert image to black and white

    //process images to get barycenter
    cv::Mat processedImg;
    //TODOM2 below are the default coordinates - to be changed
    cv::cvtColor(greyImg, processedImg, cv::COLOR_GRAY2BGR);//convert image to color (for displaying colored features)
    cv::Point targetBarycenter(160,120);
    areaPix = 1;

    //cout << "targetBarycenter is at " << targetBarycenter << endl; //print coordinates
    cv::circle(processedImg, targetBarycenter, 3, cv::Scalar(0, 0, 255), 1); 	//display red circle
    cv::circle(greyImg, targetBarycenter, 3, cv::Scalar(255, 255, 255), 1); 	//display white circle

    //show the images
    cv::namedWindow( "greyImg", cv::WINDOW_AUTOSIZE ); // Create a window for display
    cv::moveWindow( "greyImg", 340,0); // Move window
    cv::imshow( "greyImg", greyImg);                // Show the image inside it
    cv::namedWindow( "processedImg", cv::WINDOW_AUTOSIZE ); // Create a window for display
    cv::moveWindow( "processedImg", 680,0); // Move window
    cv::imshow( "processedImg", processedImg);                // Show the image inside it

    gettimeofday(&end, NULL); //get end time
    double time_spent = (end.tv_sec - start.tv_sec) * 1e3 + (end.tv_usec -
                                                             start.tv_usec) * 1e-3;
    std::cout << "ProcessImageToGetBarycenter took " << time_spent << " ms\n"; //print time spetn for processing
    return(targetBarycenter);
}

/**** Send commands (speed_L, speed_R) to motors in order to realize robot
 * velocities (v, w) for 10 seconds ****/
void SetRobotVelocities(RobotParameters rp, struct timeval startTime, const float& vel, const float& omega, char MotorCmd[15]) {
    std::cout << "The required robot velocities are v " << vel << " w " << omega << "\n\n\n";
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    long int timeSinceStart =
            ((curTime.tv_sec * 1000000 + curTime.tv_usec) -
             (startTime.tv_sec * 1000000 + startTime.tv_usec)) /
            1000;
    std::cout << "timeSinceStart = " << timeSinceStart << " ms\n";
    // Commands to be sent to the left and right wheels
    int speedL;
    int speedR;
    const float encFactor  = 0.0054;
    if (timeSinceStart < 10000) {
        speedL = (2*vel - rp.axis_length * omega) / (rp.wheel_radius * encFactor);//TODO there is a 2 * error factor on pure rotation (robot pivots twice faster than required)
        speedR = (2*vel + rp.axis_length * omega) / (rp.wheel_radius * encFactor);
        // Sends the command to the epuck motors
        sprintf(MotorCmd, "D,%d,%d", speedL, speedR);
    } else {
        sprintf(MotorCmd, "D,%d,%d", 0, 0);
    }
    std::cout << "SetRobotVelocities wheel speeds are " << speedL << " " << speedR;
}

// control robot using images from the camera
void ControlRobotWithVisualServoing(const cv::Point &baryc, float &vel, float &omega) {
    //TODOM2
    //calculate vel and omega given baryc
    vel = 0;
    omega = 0;
}
// make robot follow a wall using infrared measurements for ten seconds
void ControlRobotToFollowWall(struct timeval startTime, float &vel, float &omega) {
    // replace the lines below with commands v and w needed to follow wall
    // from xA, yA, xB et yB
    // Load these variables from files xA.txt etc...
    //std::cout << "infrared measures xA " << xA << " yA " << yA << " xB " << xB << " yB " << yB << " \n";
    // calculate equation of the line in the robot frame
    float mMur, pMur; // parametres definissant le mur dans le repere robot
    std::cout << "equation of the line in the robot frame: y = " << mMur << " x + " << pMur << " \n";
    // measure the time
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    long int timeSinceStart =
            ((curTime.tv_sec * 1000000 + curTime.tv_usec) -
             (startTime.tv_sec * 1000000 + startTime.tv_usec)) /
            1000;
    std::cout << "timeSinceStart = " << timeSinceStart << " ms\n";
    if (timeSinceStart < 10000) {
        vel = 0;
        omega = 0;
    } else {
        vel = 0;
        omega = 0;
    }
}
void InfraRedValuesToMetricDistance(RobotParameters rp, const int ProxS[10], float dist[10], Logger& log) {
    // infrared values
    for (int i = 0; i < 10; i++) {
        std::cout << "IRed " << i << " : " <<  ProxS[i] << " ";
    }
    std::cout << std::endl;
    for (int i = 0; i < 8; i++) {
        if (ProxS[i] != 0) {
            dist[i] = 1.79 * pow(ProxS[i], -.681);
        } else {
            dist[i] = rp.dist_max;
        }
    }
    for (int i = 8; i < 10; i++) {
        dist[i] = 0.1495 * exp(-.002875 * ProxS[i]) +
                  0.05551 * exp(-.0002107 * ProxS[i]);
    }  
    for (int i = 0; i < 10; i++) {
        if (dist[i] >= rp.dist_max) {
            dist[i] = 1000;// rp.distMax;
        }
        std::cout << "dist[" << i << "] -> " << dist[i] << std::endl;
    }

    //log the data
    
    log.addIn(log.file_distances, dist[0],"\t");
    log.addIn(log.file_distances, dist[1],"\t");
    log.addIn(log.file_distances, dist[2],"\t");
    log.addIn(log.file_distances, dist[3],"\t");
    log.addIn(log.file_distances, dist[4],"\t");
    log.addIn(log.file_distances, dist[5],"\t");
    log.addIn(log.file_distances, dist[6],"\t");
    log.addIn(log.file_distances, dist[7],"\t");
    log.addIn(log.file_distances, dist[8],"\t");
    log.addIn(log.file_distances, dist[9]);

}
void MetricDistanceToRobFrameCoordinates(RobotParameters rp, const float dist[10], const Pose & rPoseEnc, cv::Point2f ProxInWFr[10], float & mWall, float & pWall) {
    cv::Point2f ProxInRobFrame[10];
    for (int i = 0; i < 10; i++) {
        ProxInRobFrame[i].x = (dist[i] + rp.robot_radius) * cos(rp.theta[i]);
        ProxInRobFrame[i].y  = (dist[i] + rp.robot_radius) * sin(rp.theta[i]);
    }
    std::cout << "The robot's current pose is X " << rPoseEnc.x << " Y " << rPoseEnc.y << " Theta " << rPoseEnc.th << std::endl;
    for (int i = 0; i < 10; i++) {
        ProxInWFr[i].x = rPoseEnc.x + ProxInRobFrame[i].x * cos (rPoseEnc.th) - ProxInRobFrame[i].y * sin (rPoseEnc.th);
        ProxInWFr[i].y = rPoseEnc.y + ProxInRobFrame[i].x * sin (rPoseEnc.th) + ProxInRobFrame[i].y * cos (rPoseEnc.th);
    }
    mWall = -.5;
    pWall = 0.05;
}
