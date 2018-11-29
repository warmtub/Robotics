#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <conio.h>
#include <string>

#include "opencv.hpp"
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "4000"

using namespace std;
using namespace cv;
RNG g_rng(11111);

//return a vector of (vector of double of size 3), including x and y axis of centroid and the angle of primary axis
vector<vector<double> > find_Centroid_and_Angle(Mat srcImg, bool showImg)
{
	
	Mat g_gray;
	vector<vector<Point>> g_vContours;
	vector<Vec4i> g_vHierarchy;
	cvtColor(srcImg, g_gray, COLOR_BGR2GRAY);//convert the image to grayscale
	blur(g_gray, g_gray, Size(3, 3));//blur the image
	//threshold(g_gray, g_gray, 0, 255, THRESH_BINARY_INV | CV_THRESH_OTSU);//find the best threshold auto
	Canny(g_gray, g_gray, 260, 260 * 2, 3);//go through Canny edge detection
	if (showImg)
	{
		imshow("binarized", g_gray);
	}
	findContours(g_gray, g_vContours, g_vHierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));//finding contour
	cout << g_vContours.size() << endl;
	
	Mat drawing = Mat::zeros(g_gray.size(), CV_8UC3);//drawing the edges and center
	vector<vector<double> > Centroids_and_Angles;
	for(unsigned int i=1;i<g_vContours.size();i+=2)
	{
		Scalar color = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255));
		vector<double> centroid_and_angle;

		//Using minAreaRect to find bounding box
		vector<RotatedRect> Rect(g_vContours.size());
		for (size_t i = 0; i < g_vContours.size(); i++)
		{
			Rect[i] = minAreaRect(Mat(g_vContours[i]));
		}

		Point2f principle_axis[2];
		Point2f rect_points[4];
		Rect[i].points(rect_points);

		//Find principle axis
		if (norm(Mat(rect_points[0]), Mat(rect_points[1])) < norm(Mat(rect_points[1]), Mat(rect_points[2])))//Detecting principle_axis
		{
			principle_axis[0] = (rect_points[0] + rect_points[1]) * 0.5;
			principle_axis[1] = (rect_points[2] + rect_points[3]) * 0.5;
		}
		else
		{
			principle_axis[0] = (rect_points[0] + rect_points[3]) * 0.5;
			principle_axis[1] = (rect_points[1] + rect_points[2]) * 0.5;
		}
		if (showImg)
		{
			drawContours(drawing, g_vContours, i, color, 2, 8, g_vHierarchy, 0, Point());
			line(drawing, principle_axis[0], principle_axis[1], color, 1);
		}
		
		//Draw principle axis
		//line(frame, principle_axis[0], principle_axis[1], color, 1);
		
		Point2f center = (rect_points[0] + rect_points[2]) * 0.5;//get the center
		double angle = atan2(principle_axis[0].x - principle_axis[1].x, principle_axis[0].y - principle_axis[1].y);//get angular
		//Draw center position
		stringstream stream;
		stream << "Centroid = (" << fixed << setprecision(2) << center.x << ", " << center.y << ")\n";//Show the centroid on image
		centroid_and_angle.push_back(center.x);
		centroid_and_angle.push_back(center.y);
		//cout << stream.str();
		if (showImg)
		{
			putText(drawing, stream.str(), Point(center.x + 10, center.y + 10), 0, 0.3, color);//show the centroid on detecting image
		}
		//Draw angle position
		stream.str("");
		stream << "Angle: " << fixed << setprecision(2) << angle * 180 / 3.14 << endl;//show angle
		centroid_and_angle.push_back(angle * 180 / 3.14);
		//cout << stream.str();
		//Draw center
		if (showImg)
		{
			putText(drawing, stream.str(), Point(center.x + 10, center.y + 20), 0, 0.3, color);//show the angle on detecting image
			circle(drawing, center, 3, color, CV_FILLED);//show the center on detecting image
		}
		Centroids_and_Angles.push_back(centroid_and_angle);
	}
	if (showImg)
	{
		imshow("Drawing", drawing);
		waitKey();
	}

	return Centroids_and_Angles;
}

int sendCommand(char* sendbuf, SOCKET& ClientSocket)
{
	cout << "Sending Command: " << sendbuf << endl;
	int SendResult = 0;
	int ReceiveResult = 0;
	char recvbuf[512];
	int counter = 0;
	int RoundThreshold = 1000;

	//Send Command to Robot Arm
	SendResult = send(ClientSocket, sendbuf, strlen(sendbuf)+1, 0 );

    if (SendResult == SOCKET_ERROR)
	{
		cout<< "send failed with error: " << WSAGetLastError() << endl;
		closesocket(ClientSocket);
		WSACleanup();
		return 1;
	}

	Sleep(100);

	do
	{
		ReceiveResult = recv(ClientSocket, recvbuf, 512, 0);
		counter++;
	}while(ReceiveResult == 0 && counter < RoundThreshold);

	if(counter > RoundThreshold)
	{
		cout << "Respond Time Out" << endl;
		return 1;
	}
	else
	{
		if(!strcmp(recvbuf,"ERR"))
		{
			cout << "Invalid Command" << endl;
			return 1;
		}
	}

	return 0;
}

int __cdecl main(void) 
{
    WSADATA wsaData;
    int iResult;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    int recvbuflen = DEFAULT_BUFLEN;
    
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // No longer need server socket
    closesocket(ListenSocket);
	
	//========== Add your code below ==========//

	// Open the camera
	
	// 0. find the cordinate transform between pixel and robot
	// localize objectA
	char setSpeed[] = "SETPTPSPEED 30";
	sendCommand(setSpeed, ClientSocket);
	char goHome[] = "GOHOME";
	sendCommand(goHome, ClientSocket);
	cout << "Put the objectA into the gripper\n";
	cout << "Press ant key when done\n";
	getch();
	char closeGripper[] = "OUTPUT 48 ON";
	sendCommand(closeGripper, ClientSocket);
	cout << "Press ant key when done\n";
	getch();
	// make sure -218 dosen't hit the table
	char move2PositionA[] = "MOVL 0 550 -218 90 0 180";
	sendCommand(move2PositionA, ClientSocket);
	char openGripper[] = "OUTPUT 48 OFF";
	sendCommand(openGripper, ClientSocket);
	sendCommand(goHome, ClientSocket);
	Sleep(20000);
	
	// take a picture
	VideoCapture cap(0);
	if (!cap.isOpened())
	{
		return -1;
	}
	Mat frame;
	cap.read(frame);
	imshow("frame", frame);
	waitKey();
	vector<vector<double> > ObjectA_Centroid_and_Angle = find_Centroid_and_Angle(frame, true);
	for (int i = 0; i < ObjectA_Centroid_and_Angle.size(); ++i)
	{
		cout << "ObjectA is at (" << ObjectA_Centroid_and_Angle[i][0] << ", " << ObjectA_Centroid_and_Angle[i][1] << ") "
			 << "with angle " << ObjectA_Centroid_and_Angle[i][2] << " in pixel.\n";
	}

	// localize objectB
	Sleep(1000);
	sendCommand(move2PositionA, ClientSocket);
	sendCommand(closeGripper, ClientSocket);
	char move2PositionAhigh[] = "MOVL 0 550 -180 90 0 180";
	sendCommand(move2PositionAhigh, ClientSocket);
	char move2PositionBhigh[] = "MOVL 100 450 -180 90 0 180";
	sendCommand(move2PositionBhigh, ClientSocket);
	char move2PositionB[] = "MOVL 100 450 -218 90 0 180";
	sendCommand(move2PositionB, ClientSocket);
	sendCommand(openGripper, ClientSocket);
	sendCommand(goHome, ClientSocket);
	Sleep(20000);
	
	// take a picture
	cap.release();
	cap = VideoCapture(0);
	cap.read(frame);
	imshow("frame", frame);
	waitKey();
	vector<vector<double> > ObjectB_Centroid_and_Angle = find_Centroid_and_Angle(frame, true);
	for (int i = 0; i < ObjectB_Centroid_and_Angle.size(); ++i)
	{
		cout << "ObjectB is at (" << ObjectB_Centroid_and_Angle[i][0] << ", " << ObjectB_Centroid_and_Angle[i][1] << ") "
			 << "with angle " << ObjectB_Centroid_and_Angle[i][2] << " in pixel.\n";
	}


	// 1. Read the camera frames and open a window to show it.
	//Read in the camera
	
	
	// 2. Segment the object(s) and calculate the centroid(s) and principle angle(s).
	//Image preprocessing
	

	// 3. Use prespective transform to calculate the desired pose of the arm.

	// 4. Move the arm to the grasping pose by sendCommand() function.
	// The following lines give an example of how to send a command.
	// You can find commends in "Robot Arm Manual.pdf"
	
	// 5. Control the gripper to grasp the object.
	// The following lines give an example of how to control the gripper.
	//char closeGripper[] = "OUTPUT 48 ON";
	//sendCommand(closeGripper, ClientSocket);
	Sleep(1000);
	
	//========== Add your code above ==========//
	
	system("pause"); 
		
	// shutdown the connection since we're done
    iResult = shutdown(ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();
		
    return 0;
}
