/*********  compile and link with command *********
g++ lidar.cpp -lwiringPi -o lidartest
**********  run with  *****************************
./spitest
********** break the program with *****************
control-c
***************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <algorithm>
#include <vector>

#include <Eigen/Dense>
//#include "coxRemade.h"


using namespace std;

Eigen::Matrix<double, 1, 500> DIS;
Eigen::Matrix<double, 1, 500> ang;
bool needData;
int Array_full_flag = 0;


void lidarRead(void) {

	int Index = 0;
	int Received_Bytes = 0;
	char recv_buffer[10];
	int sockfd, newsockfd;
	socklen_t clilen;
	struct sockaddr_in serv_addr, cli_addr;
	int iQuality;
	int iDistance;
	int iAngle;


	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		puts("ERROR opening socket");
	else
		puts("open socket");
	int start = system("python3 start.py &");
	bzero((char *) &serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(9888);
	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
		puts("ERROR on binding");
	listen(sockfd, 1);
	clilen = sizeof(cli_addr);
	newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if (newsockfd < 0)
		puts("ERROR on accept");
	else
		puts("Client connected!");


	while (1) {
		Received_Bytes = recv(newsockfd, recv_buffer, 5, 0);
		if (recv_buffer[0] == 0xA5) {
			Received_Bytes = recv(newsockfd, recv_buffer, 5, 0);
			if (needData){
				iQuality = recv_buffer[0] >> 2;
				iAngle = (recv_buffer[1] >> 1) + (recv_buffer[2] << 8) >> 7;
				iDistance = (recv_buffer[3]) + (recv_buffer[4] << 8) >> 2;
				if (Array_full_flag == 0){
					if(iQuality == 15 && iDistance != 0){
						DIS(0, Index) = iDistance; // /4
						ang(0, Index) = (double)iAngle * -1; // * -0.000136353
						Index++;
						if (Index == 500){
							Array_full_flag = 1;
							//needData = false;
							Index = 0;
						}
					}
				}

			}
		}

	}
	close(newsockfd);
	close(sockfd);
	int stop = system("python3 stop.py &");
};
