#include "RobotDriver.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <pthread.h>
#include <sstream>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


#define INVALID_SOCKET -1
#define SOCKET_ERROR -1

#include "datafile.hpp"

#define HERE(X) std::cout << "HI THERE " << X <<  std::endl

EPuckV1Driver::EPuckV1Driver(Robot& robot, char** arg) : RobotDriver(robot, arg) {
    stop_threads_ = false;
    camera_active_ = true;
    is_the_connection_lost_ = 0;
}

EPuckV1Driver::~EPuckV1Driver() {
    log().CloseAll();
    stop_threads_ = true;
    // Send a zero velocity command before exiting
    setWheelCommands(robot());
    sendMotorAndLEDCommandToRobot(MotorCommand_);
    std::cout << "All good, mate\n";

    pthread_join(id_camera_reception_thread_, NULL);

    //signal(SIGINT, nullptr);

    closeSocket(camera_socket_);

    closeSocket(command_sending_socket_);
    closeSocket(sensor_receiving_socket_);
}

bool EPuckV1Driver::init() {
    initCamera(robot().ip); // sends camera state info to epuck

    openSensorReceivingSocket();
    openCommandSendingSocket(robot().ip);

    openCameraSocket();

    if (pthread_create(&id_camera_reception_thread_, NULL, cameraThreadFunc, this) == -1) {
        printf("Error while creating the camera reception thread!\n");
        return false;
    } else {
         printf("Camera reception thread succesfully created!\n");
    }

    std::cout << "\nINIT \n";

    cnt_iter = 1; // to get the current iteration
    gettimeofday(&startTime, NULL); // get starting time
    init_pose_.setPose(.32, 0., M_PI);

    return true;
}

// TODO
void EPuckV1Driver::read() {
    //robot().vision_sensors = showAndSaveRobotImage(img_data_.msg, cnt_iter); 
    std::cout << "\033[1;36m";//write in bold cyan
    std::cout << "\nSTART ITERATION " << cnt_iter <<" \n";
    std::cout << "\033[0m";//reset color
    gettimeofday(&prevTime, NULL);
    //show and save image
    if (camera_active_ == true) {
        rob_img_ = showAndSaveRobotImage(img_data_.msg, cnt_iter);
    }
    gettimeofday(&curTime, NULL);
    timeSinceStart = (curTime.tv_sec - prevTime.tv_sec) * 1e3 + (curTime.tv_usec - prevTime.tv_usec) * 1e-3;
    //timeSinceStart = ((curTime.tv_sec * 1000000 + curTime.tv_usec) - (prevTime.tv_sec * 1000000 + prevTime.tv_usec)) / 1000;
    std::cout << "IP\ttimeSinceStart = " << timeSinceStart << " ms\n";
    gettimeofday(&prevTime, NULL);


    if(cnt_iter == 1) {
        cur_pose_from_enc_ = init_pose_;
        cur_pose_from_vis_ = init_pose_;
    } else {
        cur_pose_from_enc_ = getCurrPoseFromEncoders(robot().parameters, prev_pose_from_enc_, encoder_left_, encoder_right_, prev_encoder_left_, prev_encoder_right_, log());
    }
    float areaPix;
    cv::Point baryc = processImageToGetBarycenter(rob_img_ , areaPix);
    cur_pose_from_vis_ = getCurrPoseFromVision(baryc, cur_pose_from_enc_.th, areaPix, log());
    prev_pose_from_enc_ = cur_pose_from_enc_;
    prev_pose_from_vis_ = cur_pose_from_vis_;

    receiveSensorMeasures(); // receive data from encoders and proximity sensors///////////////////////////////////////////////////////////////////////////////////////////////////////////
    splitSensorMeasures(); // splits measures and converts them to integer///////////////////////////////////////////////////////////////////////////////////////////////////////////
    float dist[10];
    infraRedValuesToMetricDistance(robot().parameters, prox_sensors_, dist, log());
    cv::Point2f ProxInWFrame[10];
    float mRob, pRob, mWorld, pWorld;
    convertIRPointsForWallFollowing(robot().parameters, dist, cur_pose_from_enc_, ProxInWFrame, mRob, pRob, mWorld, pWorld);
    drawMapWithRobot(robot().parameters, cur_pose_from_enc_, cur_pose_from_vis_, ProxInWFrame, mWorld, pWorld);

    
    saveData(log());
    closeFilesIfConnectionLost();


};


void EPuckV1Driver::sendCmd() {
      /////////////////////////
     /////////TIME////////////
    /////////////////////////

    // Sends the command to the epuck motors
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    long int timeSinceStart =
        ((curTime.tv_sec * 1000000 + curTime.tv_usec) -
         (startTime.tv_sec * 1000000 + startTime.tv_usec)) /
        1000;
    std::cout << "timeSinceStart = " << timeSinceStart << " ms\n";

    //if (timeSinceStart < 10000) {
        // Sends the command to the epuck motors
        sprintf(MotorCommand_, "D,%d,%d", (int)robot().wheels_command.left_velocity, (int)robot().wheels_command.right_velocity);
    //} else {
    //    sprintf(MotorCommand_, "D,%d,%d", 0, 0);
    //}
    
    sendMotorAndLEDCommandToRobot(MotorCommand_);

    cnt_iter++;

    gettimeofday(&curTime, NULL);
    timeSinceStart = (curTime.tv_sec - prevTime.tv_sec) * 1e3 + (curTime.tv_usec - prevTime.tv_usec) * 1e-3;

    std::cout << "proc\ttimeSinceStart = " << timeSinceStart <<" ms\n";
	gettimeofday(&prevTime, NULL);
};

// Empty (not needed in EPuckV1Driver)
void EPuckV1Driver::getVisionSensor(Robot& robot) {
    std::cout << "";
}


/**** fonction initialisation Camera ****/
void EPuckV1Driver::initCamera(const std::string& epuck_ip) {
    initSocketOpening(robot().ip); // ouverture de la socket d'initialisation
    int Send;
    char CommandeInit[2];
    /* Mise en place de la commande d'init */
    if (camera_active_ == true) {
        sprintf(CommandeInit, "1");
    }
    if (camera_active_ == false) {
        sprintf(CommandeInit, "0");
    }
    std::cout << "commande Init = %s\n", CommandeInit;
    /* Envoi commande d'init */
    Send = sendto(sock_init_, CommandeInit, sizeof(CommandeInit), 0,
                  (struct sockaddr*)&sockaddrin_init_, sizeof(sockaddrin_init_));
    if (Send < 0) {
        std::cout << "erreur sur l'envoi de l'Init\n";
    } else if (Send == 0) {
        std::cout << "Envoi vide \n";
    }
    closeSocket(sock_init_); // fermeture de la socket d'initialisation
    // HERE (by robin) waiting a little time to avoid synchronizatipon problem
    // with server (socket to receive commands not created yet)
    std::cout << "waiting server to be ready...\n";
    std::this_thread::sleep_for(std::chrono::microseconds(2000));
}

// Functions for opening sockets
void EPuckV1Driver::initSocketOpening(const std::string& epuck_ip) {
    std::cout << "Creating socket Init :\n\r";
    static const int PortsockInit = 1029;
    sock_init_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_init_ == INVALID_SOCKET) {
        std::cout << "Desole, je ne peux pas creer la socket port: " << PortsockInit << "\n";
    } else {
        std::cout << "socket port " << PortsockInit << " : OK \n";
        /* Lie la socket � une ip et un port d'�coute */
        // callee (epuck) -> distant socket where I need to send to epuck
        sockaddrin_init_.sin_family = AF_INET;
        sockaddrin_init_.sin_addr.s_addr =
            inet_addr(epuck_ip.c_str()); // IP de l'epuck
        sockaddrin_init_.sin_port =
            htons(PortsockInit); // Ecoute ou emet sur le PORT
        // caller (local PC)
        local_sockaddrin_init_.sin_family = AF_INET;
        local_sockaddrin_init_.sin_addr.s_addr =
            htonl(INADDR_ANY); // IP de l'epuck
        local_sockaddrin_init_.sin_port =
            htons(PortsockInit); // Ecoute ou emet sur le PORT
        int erreur = bind(sock_init_, (SOCKADDR*)&local_sockaddrin_init_,
                          sizeof(local_sockaddrin_init_));
        if (erreur == SOCKET_ERROR) {
            perror("ERROR Sock INIT :");
            std::cout << "Echec du Bind de la socket port : " << PortsockInit <<" \n";
        } else {
            std::cout << "bind : OK port : "<< PortsockInit <<" \n";
        }
    }
}

void EPuckV1Driver::openCameraSocket() {
    std::cout << "Creation Camera socket :\n\r";
    camera_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    static const int PortsockCamera = 1026;

    if (camera_socket_ == INVALID_SOCKET) {
        std::cout << "Desole, je ne peux pas creer la socket port: "<< PortsockCamera << " \n";
    } else {
        std::cout << "socket port " << PortsockCamera << " : OK \n";
        /* Lie la socket � une ip et un port d'�coute */
        sockaddrin_camera_.sin_family = AF_INET;
        sockaddrin_camera_.sin_addr.s_addr = htonl(INADDR_ANY); // IP
        sockaddrin_camera_.sin_port =
            htons(PortsockCamera); // Ecoute ou emet sur le PORT
        int erreur = bind(camera_socket_, (SOCKADDR*)&sockaddrin_camera_,
                          sizeof(sockaddrin_camera_));
        if (erreur == SOCKET_ERROR) {
            std::cout << "Echec du Bind de la socket port : " << PortsockCamera << " \n";
        } else {
            std::cout << "bind : OK port : " << PortsockCamera << " \n";
        }
    }
}
void EPuckV1Driver::openSensorReceivingSocket() {
    std::cout << "Creation socket ReceptionCapteurs :\n\r";
    static const int PortsockReceptionCapteurs = 1028;
    sensor_receiving_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sensor_receiving_socket_ == INVALID_SOCKET) {
        std::cout << "Desole, je ne peux pas creer la socket port: " << PortsockReceptionCapteurs << " \n";
    } else {
        std::cout << "socket port %d : " << PortsockReceptionCapteurs <<" \n";
        /* Lie la socket � une ip et un port d'�coute*/
        sockaddrin_reception_capteurs_.sin_family = AF_INET;
        sockaddrin_reception_capteurs_.sin_addr.s_addr = htonl(INADDR_ANY); // IP
        sockaddrin_reception_capteurs_.sin_port =
            htons(PortsockReceptionCapteurs); // Ecoute ou emet sur le PORT
        int erreur =
            bind(sensor_receiving_socket_, (SOCKADDR*)&sockaddrin_reception_capteurs_,
                 sizeof(sockaddrin_reception_capteurs_));
        if (erreur == SOCKET_ERROR) {
            std::cout << "Echec du Bind de la socket port : "<< PortsockReceptionCapteurs << " \n";
        } else {
            std::cout << "bind : OK port : " << PortsockReceptionCapteurs << " \n";
        }
    }
}

inline int sign(float val) {
    if (val < 0)
        return -1;
    if (val == 0)
        return 0;
    return 1;
}



/* Socket EnvoieCommandes */
void EPuckV1Driver::openCommandSendingSocket(const std::string& epuck_ip) {
    std::cout << "Creation socket EnvoieCommandes :\n\r";
    static const int PortsockEnvoieCommandes = 1027;
    command_sending_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (command_sending_socket_ == INVALID_SOCKET) {
        std::cout << "Desole, je ne peux pas creer la socket port: " << PortsockEnvoieCommandes << " \n";
    } else {
        std::cout << "socket port " << PortsockEnvoieCommandes << " : OK \n";
        /* Lie la socket � une ip et un port d'�coute */
        // callee (epuck) -> distant socket where I need to send to epuck
        sockaddrin_envoie_commandes_.sin_family = AF_INET;
        sockaddrin_envoie_commandes_.sin_addr.s_addr =
            inet_addr(epuck_ip.c_str()); // IP
        sockaddrin_envoie_commandes_.sin_port =
            htons(PortsockEnvoieCommandes); // Ecoute ou emet sur le PORT
        // caller (local PC)
        local_sockaddrin_envoie_commandes_.sin_family = AF_INET;
        local_sockaddrin_envoie_commandes_.sin_addr.s_addr =
            htonl(INADDR_ANY); // IP de l'epuck
        local_sockaddrin_envoie_commandes_.sin_port =
            htons(PortsockEnvoieCommandes); // Ecoute ou emet sur le PORT
        int erreur = bind(command_sending_socket_,
                          (SOCKADDR*)&local_sockaddrin_envoie_commandes_,
                          sizeof(local_sockaddrin_envoie_commandes_));
        if (erreur == SOCKET_ERROR) {
            std::cout << "Echec du Bind de la socket port : " << PortsockEnvoieCommandes << " \n";
        } else {
            std::cout << "bind : OK port : " << PortsockEnvoieCommandes << " \n";
        }
    }
}
/**** Fonction de fermeture de socket ****/
void EPuckV1Driver::closeSocket(int NOM_SOCKET) {
    shutdown(NOM_SOCKET, 2); // Ferme la session d'emmission et d'�coute
    close(NOM_SOCKET);       // Ferme la socket
}
void EPuckV1Driver::sendMotorAndLEDCommandToRobot(const char MotorCmd[15]) {
    char MotorAndLEDCommand[23], LEDCommand[9];
    // prepare LED commands
    char Led_1, Led_2, Led_3, Led_4, Led_5, Led_6, Led_7, Led_8;
    Led_1 = '1';
    Led_2 = '1';
    Led_3 = '1';
    Led_4 = '1';
    Led_5 = '1';
    Led_6 = '1';
    Led_7 = '1';
    Led_8 = '1'; // 1 for ON, 0 for OFF
    sprintf(LEDCommand, "%c%c%c%c%c%c%c%c", Led_1, Led_2, Led_3, Led_4, Led_5,
            Led_6, Led_7, Led_8);
    sprintf(MotorAndLEDCommand, "%s%s", LEDCommand, MotorCmd);
    std::cout << "Command to be sent: " << MotorAndLEDCommand <<"\n";
    int Send_;
    Send_ = sendto(command_sending_socket_, MotorAndLEDCommand, 23, 0,
                  (struct sockaddr*)&sockaddrin_envoie_commandes_,
                  sizeof(sockaddrin_envoie_commandes_));
    if (Send_ < 0) {
        std::cout << "Error sending commands\n";
    } else if (Send_ == 0) {
        std::cout << "Error nothing sent\n";
    }
}
void EPuckV1Driver::receiveSensorMeasures() {
    int BytesReception;
    socklen_t Size = sizeof(sockaddrin_reception_capteurs_);
    memset(all_sensors_, 0, 100);

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    
    fd_set socks;
    FD_ZERO(&socks);
    FD_SET(sensor_receiving_socket_, &socks);
    if (select(sensor_receiving_socket_ + 1, &socks, NULL, NULL, &timeout)) {
        BytesReception =
            recvfrom(sensor_receiving_socket_, all_sensors_, 100, 0,
                     (SOCKADDR*)&sockaddrin_reception_capteurs_, &Size);
        if (BytesReception < 0) {
            all_sensors_[0] = 0;
            std::cout << "SORRY No Data received\n";
        } else {
            all_sensors_[BytesReception] = 0;
            if (BytesReception > 0) {
            } else {
                std::cout << "WARNING No Data received!\n";
            }
        }
    } else {
        std::cout << "TIMEOUT Sensor data reading\n";
    }
}
void EPuckV1Driver::splitSensorMeasures() {
    if (all_sensors_[0] == 0) {
        std::cout << "nothing received, no message to manage\n";
        return;
    }
    int SizeBufferto, SizeCapteurs,
        index = -1; // taille des capteurs et emplacement du caractere pour la
                    // d�coupe : index
    char CharEncodeur[2][20], CharProx[10][5];
    char* ptr_pos = NULL;
    char* bufferto;
    /* Preparation de la memoire */
    for (int i = 0; i++; i < 2) {
        memset(CharEncodeur[i], 0, 20);
    }
    for (int i = 0; i++; i < 10) {
        memset(CharProx[i], 0, 5);
    }

    /* Decoupage progressif des valeurs des capteurs */
    bufferto = (char*)calloc(72, sizeof(char));

    ptr_pos = strstr(all_sensors_, "q"); // on cherche l'index o� se trouve q
    ptr_pos ? index = ptr_pos - all_sensors_ + 2 : index = 0;

    memcpy(bufferto, &all_sensors_[index], SizeBufferto = 72);
    ptr_pos =
        strstr(bufferto,
               ","); // on cherche l'index o� se trouve la premi�re virgule
    ptr_pos ? index = ptr_pos - bufferto : index = 0;
    memcpy(CharEncodeur[0], bufferto,
           index); // on copie depuis le d�but du Buffer sur une longueur
                   // correspondant � l'index cherch� pr�c�demment

    memcpy(all_sensors_, &bufferto[index + 1],
           SizeCapteurs =
               72 - (index + 1)); // on "enleve" la partie d�j� trait�e
    ptr_pos = strstr(all_sensors_, "n");
    ptr_pos ? index = ptr_pos - all_sensors_ : index = 0;
    memcpy(CharEncodeur[1], all_sensors_, index);

    memcpy(bufferto, &all_sensors_[index + 2],
           SizeBufferto = SizeCapteurs - (index + 2));
    ptr_pos = strstr(bufferto, ",");
    ptr_pos ? index = ptr_pos - bufferto : index = 0;
    memcpy(CharProx[0], bufferto, index);

    memcpy(all_sensors_, &bufferto[index + 1],
           SizeCapteurs = SizeBufferto - (index + 1));
    ptr_pos = strstr(all_sensors_, ",");
    ptr_pos ? index = ptr_pos - all_sensors_ : index = 0;
    memcpy(CharProx[1], all_sensors_, index);

    memcpy(bufferto, &all_sensors_[index + 1],
           SizeBufferto = SizeCapteurs - (index + 1));
    ptr_pos = strstr(bufferto, ",");
    ptr_pos ? index = ptr_pos - bufferto : index = 0;
    memcpy(CharProx[2], bufferto, index);

    memcpy(all_sensors_, &bufferto[index + 1],
           SizeCapteurs = SizeBufferto - (index + 1));
    ptr_pos = strstr(all_sensors_, ",");
    ptr_pos ? index = ptr_pos - all_sensors_ : index = 0;
    memcpy(CharProx[3], all_sensors_, index);

    memcpy(bufferto, &all_sensors_[index + 1],
           SizeBufferto = SizeCapteurs - (index + 1));
    ptr_pos = strstr(bufferto, ",");
    ptr_pos ? index = ptr_pos - bufferto : index = 0;
    memcpy(CharProx[4], bufferto, index);

    memcpy(all_sensors_, &bufferto[index + 1],
           SizeCapteurs = SizeBufferto - (index + 1));
    ptr_pos = strstr(all_sensors_, ",");
    ptr_pos ? index = ptr_pos - all_sensors_ : index = 0;
    memcpy(CharProx[5], all_sensors_, index);

    memcpy(bufferto, &all_sensors_[index + 1],
           SizeBufferto = SizeCapteurs - (index + 1));
    ptr_pos = strstr(bufferto, ",");
    ptr_pos ? index = ptr_pos - bufferto : index = 0;
    memcpy(CharProx[6], bufferto, index);

    memcpy(all_sensors_, &bufferto[index + 1],
           SizeCapteurs = SizeBufferto - (index + 1));
    ptr_pos = strstr(all_sensors_, ",");
    ptr_pos ? index = ptr_pos - all_sensors_ : index = 0;
    memcpy(CharProx[7], all_sensors_, index);

    memcpy(bufferto, &all_sensors_[index + 1],
           SizeBufferto = SizeCapteurs - (index + 1));
    ptr_pos = strstr(bufferto, ",");
    ptr_pos ? index = ptr_pos - bufferto : index = 0;
    memcpy(CharProx[8], bufferto, index);

    memcpy(all_sensors_, &bufferto[index + 1],
           SizeCapteurs = SizeBufferto - (index + 1));
    ptr_pos = strstr(all_sensors_, "\r");
    ptr_pos ? index = ptr_pos - all_sensors_ : index = 0;
    memcpy(CharProx[9], all_sensors_, index);

    /* conversion des Char en Int */
    for (int i = 0; i < 10; i++) {
        prox_sensors_[i] = atoi(CharProx[i]);
    }
    prev_encoder_left_ = encoder_left_;
    prev_encoder_right_ = encoder_right_;
    encoder_left_ = atoi(CharEncodeur[0]);
    encoder_right_ = atoi(CharEncodeur[1]);
}

/*****************/
/**** Threads ****/
/*****************/
/**** Thread for receiving camera data ****/
void* EPuckV1Driver::cameraReceptionThread(void* arg) {
    std::cout << "Executing thread for receiving camera data\n\r";
    static const int img_msg_size = 57600;
    static const int img_msg_header = 8;
    int num_bytes = -2;
    
    socklen_t sinsize = sizeof(sockaddrin_camera_);
    unsigned char* buffer = (unsigned char*)calloc(
        img_msg_size + img_msg_header, sizeof(unsigned char));
    memset(img_data_.msg, 0x0, img_msg_size);
    char idImg_buf[2], idBlock_buf[2];
    memset(idImg_buf, 0, 2);
    memset(idBlock_buf, 0, 2);
    memset(buffer, 0, img_msg_size + img_msg_header);
    int i = 0;
    bool GotData = false;
    
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    fd_set socks;
    FD_ZERO(&socks);
    FD_SET(camera_socket_, &socks);
    while ( stop_threads_ == false ) {
		// int ret = select(CameraSocket + 1, &socks, NULL, NULL, &timeout);
        // if (ret > 0) {
            num_bytes = recvfrom(camera_socket_, (char*)buffer,
                                 img_msg_size + img_msg_header, 0,
                                 (SOCKADDR*)&sockaddrin_camera_, &sinsize);
            //std::cout << "received n bytes " << num_bytes << " " << i++ << " times\n";
            if (num_bytes < 0) {
                if (!GotData) {
                    std::cout << "Warning: no bytes received\n";
                    GotData = true;
                }
            } else {
                GotData = false;
                memcpy(idImg_buf, &buffer[0], 2);
                // std::cout << "idimg : " << idImg_buf << std::endl;
                img_data_.id_img = atoi(idImg_buf);
				//std::cout << "id_img: " << imgData.id_img << std::endl;
                memcpy(idBlock_buf, &buffer[3], 1);
                // std::cout << "idBlock : " << idBlock_buf << std::endl;
                img_data_.id_block = atoi(idBlock_buf);
				//std::cout << "id_block: " << imgData.id_block << std::endl;
                memcpy(&img_data_.msg[(img_data_.id_block) * img_msg_size],
                       &buffer[8], img_msg_size);
				//for(int i=0; i<10; ++i) {
				//	std::cout << (int)buffer[8+i] << " ";
				//}
                
		        std::cout << std::endl;
                num_bytes = -1;
            }
        /*}
		else if(ret < 0) {
			std::cout << "Select error camera thread: "<< errno <<"\n";
		}*/
    }
	std::cout << "Exiting camera thread\n";
    // close the thread
    (void)arg;
    pthread_exit(NULL);
}
// SPECIFIC FUNCTIONS
void EPuckV1Driver::saveData(Logger& log) {

    for(int i = 0; i<8 ; ++i) {
        robot().proximity_sensors.ir[i] = prox_sensors_[i];
    }

    log.addIn(log.file_eg, encoder_left_);
    log.addIn(log.file_ed, encoder_right_);
    log.addIn(log.file_ir[0], robot().proximity_sensors.ir[0]);
    log.addIn(log.file_ir[1], robot().proximity_sensors.ir[1]);
    log.addIn(log.file_ir[2], robot().proximity_sensors.ir[2]);
    log.addIn(log.file_ir[3], robot().proximity_sensors.ir[3]);
    log.addIn(log.file_ir[4], robot().proximity_sensors.ir[4]);
    log.addIn(log.file_ir[5], robot().proximity_sensors.ir[5]);
    log.addIn(log.file_ir[6], robot().proximity_sensors.ir[6]);
    log.addIn(log.file_ir[7], robot().proximity_sensors.ir[7]);
    /*
    log.addIn(log.file_p8, prox_sensors_[8]);
    log.addIn(log.file_p9, prox_sensors_[9]);
    */
}

void EPuckV1Driver::closeFilesIfConnectionLost() {
    if (prox_sensors_prev_ ==  robot().proximity_sensors.ir) {
        is_the_connection_lost_++;
    }
    prox_sensors_prev_ =  robot().proximity_sensors.ir ;

    if(is_the_connection_lost_ == 3) {
        log().CloseAll();
    }
}