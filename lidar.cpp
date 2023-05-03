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
using namespace std;

class lidarData {

public:
    double distanceArray[5000];
    double angleArray[5000];
    double qualityArray[5000];

    double* returnDistanceData() {
        return distanceArray;
    }

    double* returnAngleData() {
        return angleArray;
    }

    double* returnQualityData() {
        return qualityArray;
    }

    int main(void) {

        FILE *data;

        int Index = 0;
        int Received_Bytes = 0;
        char recv_buffer[10];
        int sockfd, newsockfd;
        socklen_t clilen;
        struct sockaddr_in serv_addr, cli_addr;
        int iQuality;
        int iDistance;
        int iAngle;
        int Counter = 0;
        int Array_full_flag = 0;
        int Array_Size = 5000;
        int Need_Laser_Data = 1;
        double Dist_vec[5000];
        int Qual_vec[5000];
        double Angle_vec[5000];


        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
            puts("ERROR opening socket");
        else
            puts("open socket");
        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(9888);
        if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
            puts("ERROR on binding");
        cout << '1' << endl;
        listen(sockfd, 1);
        cout << '2' << endl;
        clilen = sizeof(cli_addr);
        cout << '3' << endl;
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
        cout << '4' << endl;
        if (newsockfd < 0)
            puts("ERROR on accept");
        else
            puts("Client connected!");

        data = fopen("test.txt", "w");
        while (1) {
            Received_Bytes = recv(newsockfd, recv_buffer, 5, 0);
            if (recv_buffer[0] == 0xA5) {
                Received_Bytes = recv(newsockfd, recv_buffer, 5, 0);
                if (Index < Array_Size) {

                    iQuality = recv_buffer[0] >> 2;
                    iAngle = (recv_buffer[1] >> 1) + (recv_buffer[2] << 8);
                    iDistance = (recv_buffer[3]) + (recv_buffer[4] << 8);
                    Dist_vec[Index] = (iDistance / 4.0);
                    Angle_vec[Index] = -((double) iAngle) * 0.000136353;
                    Qual_vec[Index] = iQuality;
                    if (iDistance > 50)
                        Index++;

                } else
                    break;
            }
        }

        printf("tjo");
        for (int i = 0; i < 5000; i++){
            distanceArray[i] = Dist_vec[i];
            angleArray[i] = Angle_vec[i];
            qualityArray[i] = Qual_vec[i];
        }
        fclose(data);
        close(newsockfd);
        close(sockfd);
    }
};