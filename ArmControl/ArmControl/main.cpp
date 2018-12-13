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
#include <sstream>


#include "opencv.hpp"
// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "4000"
#define ARM1 True

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
#ifdef ARM1
	Canny(g_gray, g_gray, 160, 160 * 2, 3);//go through Canny edge detection
#else
	Canny(g_gray, g_gray, 260, 260 * 2, 3);//go through Canny edge detection
#endif
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
	
	//Express general robot arm commands
	char openGripper[] = "OUTPUT 48 OFF";
	char closeGripper[] = "OUTPUT 48 ON";
	char goHome[] = "GOHOME";
	//char move2PositionCal1[] = ss.str();
	//char move2PositionCal2[] = strcat("MOVL ", Xcal2, " ", Ycal2, " ", Zcal1, " ",  Acal1, " 0 180") ;
#ifdef ARM1
	char move2PositionCal1[] = "MOVL 0 450 -255 90 0 180";
	char move2PositionCal2[] = "MOVL -100 550 -255 90 0 180";
	char move2PositionCal1High[] = "MOVL 0 450 -215 90 0 180";
	char move2PositionCal2High[] = "MOVL -100 550 -215 90 0 180";
#else
	char move2PositionCal1[] = "MOVL 0 550 -230 90 0 180";
	char move2PositionCal2[] = "MOVL 100 450 -230 90 0 180";
	char move2PositionCal1High[] = "MOVL 0 550 -190 90 0 180";
	char move2PositionCal2High[] = "MOVL 100 450 -190 90 0 180";
#endif

	//Set speed
	char setSpeedRate[] = "SETSPEEDRATE 100"; //speed rate (speed multiplier in %)
	sendCommand(setSpeedRate, ClientSocket);
	char setTpSpeed[] = "SETPTPSPEED 20"; //speed when going home
	sendCommand(setTpSpeed, ClientSocket);
	char setSpeed[] = "SETLINESPEED 30"; //Speed in cartesian space 
	sendCommand(setSpeed, ClientSocket);

	//Robot go home
	sendCommand(goHome, ClientSocket);
	
	//For camera
	VideoCapture cap;
	Mat frame;
	Rect myROI(110, 0, 530, 400);

	// 0. find the cordinate transform between pixel and robot
	vector<vector<double> > ObjectA_Centroid_and_Angle, ObjectB_Centroid_and_Angle;
	cout << "Press 1 to start calibration, 2 to input calibration position:\n";
	char a = getch();
	if (a == '1')
	{
		cout << "Start calibration process";
		//Ask user to put object into gripper
		cout << "Put the objectA into the gripper\n";
		cout << "Press any key when done\n";
		getch();
		sendCommand(closeGripper, ClientSocket);
		
		cout << "Please leave the workspace\n";
		cout << "Press any key when done\n";
		getch();

		//Place object on posCal1 then open gripper and then go home position
		sendCommand(move2PositionCal1, ClientSocket);
		sendCommand(openGripper, ClientSocket);
		sendCommand(goHome, ClientSocket);
	
		//Waits 25s before taking picture TODO: This could be improved
		Sleep(25000);
		
		// take a picture
#ifdef ARM1
		cap = VideoCapture(10);
#else
		cap = VideoCapture(0);
#endif
		cap.read(frame);
#ifdef ARM1
		frame = frame(myROI);
#endif
		imshow("frame", frame);
		waitKey();
		ObjectA_Centroid_and_Angle = find_Centroid_and_Angle(frame, true);
		for (int i = 0; i < ObjectA_Centroid_and_Angle.size(); ++i)
		{
			cout << "ObjectA is at (" << ObjectA_Centroid_and_Angle[i][0] << ", " << ObjectA_Centroid_and_Angle[i][1] << ") "
				 << "with angle " << ObjectA_Centroid_and_Angle[i][2] << " in pixel.\n";
		}
		
		// localize objectB
		sendCommand(move2PositionCal1, ClientSocket);
		sendCommand(closeGripper, ClientSocket);
		sendCommand(move2PositionCal1High, ClientSocket);
		sendCommand(move2PositionCal2High, ClientSocket);
		sendCommand(move2PositionCal2, ClientSocket);
		sendCommand(openGripper, ClientSocket);
		sendCommand(goHome, ClientSocket);
		Sleep(30000);
	
		// take a picture
		cap.release();
#ifdef ARM1
		cap = VideoCapture(10);
#else
		cap = VideoCapture(0);
#endif
		cap.read(frame);
#ifdef ARM1
		frame = frame(myROI);
#endif
		imshow("frame", frame);
		waitKey();
		ObjectB_Centroid_and_Angle = find_Centroid_and_Angle(frame, true);
		for (int i = 0; i < ObjectB_Centroid_and_Angle.size(); ++i)
		{
			cout << "ObjectB is at (" << ObjectB_Centroid_and_Angle[i][0] << ", " << ObjectB_Centroid_and_Angle[i][1] << ") "
				 << "with angle " << ObjectB_Centroid_and_Angle[i][2] << " in pixel.\n";
		}
	}
	else if (a == '2')
	{
		cout << "Input real position of ObjectA:\n";
		double x, y;
		double angle = 0;
		cin >> x >> y;
		vector<double> Centroid_and_Angle;
		Centroid_and_Angle.push_back(x);
		Centroid_and_Angle.push_back(y);
		Centroid_and_Angle.push_back(angle);
		ObjectA_Centroid_and_Angle.push_back(Centroid_and_Angle);
		Centroid_and_Angle.clear();
		cout << "Input real position of ObjectB:\n";
		cin >> x >> y;
		Centroid_and_Angle.push_back(x);
		Centroid_and_Angle.push_back(y);
		Centroid_and_Angle.push_back(angle);
		ObjectB_Centroid_and_Angle.push_back(Centroid_and_Angle);
	}
	else
	{
		cout << "Invalid input";
		return -1;
	}
	
	//Ask user to place the N blocks to be stacked
	cout << "Press any key when N blocks to be stacked are placed\n";
	getch();

	// 1. Read the camera frames and open a window to show it.
	//Read in the camera
	//Find centroid and angle for the N blocks to be stacked 
	cap.release();
#ifdef ARM1
	cap = VideoCapture(10);
#else
	cap = VideoCapture(0);
#endif
	cap.read(frame);
#ifdef ARM1
		frame = frame(myROI);
#endif
	imshow("frame", frame);
	waitKey();
	// 2. Segment the object(s) and calculate the centroid(s) and principle angle(s).
	//Image preprocessing
	vector<vector<double> > Objectstacked_Centroid_and_Angle = find_Centroid_and_Angle(frame, true);
	for (int i = 0; i < Objectstacked_Centroid_and_Angle.size(); ++i)
	{
		cout << "ObjectB is at (" << Objectstacked_Centroid_and_Angle[i][0] << ", " << Objectstacked_Centroid_and_Angle[i][1] << ") "
			 << "with angle " << Objectstacked_Centroid_and_Angle[i][2] << " in pixel.\n";
	}

	//stack objects
	
	// 3. Use prespective transform to calculate the desired pose of the arm.
	float xcal1 = ObjectA_Centroid_and_Angle[0][0];
	float ycal1 = ObjectA_Centroid_and_Angle[0][1];
	float xcal2 = ObjectB_Centroid_and_Angle[0][0];
	float ycal2 = ObjectB_Centroid_and_Angle[0][1];
#ifdef ARM1
	float Xcal1 = 0;
	float Xcal2 = -100;
	float Ycal1 = 450;
	float Ycal2 = 550;
	float Zcal1 = -255;
#else
	float Xcal1 = 0;
	float Xcal2 = 100;
	float Ycal1 = 550;
	float Ycal2 = 450;
	float Zcal1 = -230;
#endif
	//int K = (Xcal1-Xcal2)/(xcal1-xcal2);
	//float K = (0-100)/(ObjectA_Centroid_and_Angle[0][0]-ObjectB_Centroid_and_Angle[0][0]);
	float Kx = (Xcal1-Xcal2)/(xcal1-xcal2);
	float Ky = (Ycal1-Ycal2)/(ycal1-ycal2);
	//cout << "K =" << K << endl;
	//int x[ObjectB_Centroid_and_Angle.size()];
	//int y[ObjectB_Centroid_and_Angle.size()];
	//int th[ObjectB_Centroid_and_Angle.size()];
	float x[10], X[10];
	float y[10], Y[10];
	float th[10];
	x[0] = Objectstacked_Centroid_and_Angle[0][0];
	y[0] = Objectstacked_Centroid_and_Angle[0][1];
	th[0] = Objectstacked_Centroid_and_Angle[0][2]+90;
	X[0] = Xcal1+Kx*(x[0]-xcal1);
	Y[0] = Ycal1+Ky*(y[0]-ycal1);

	float Hblock = 40;
	for (int i = 1; i < Objectstacked_Centroid_and_Angle.size(); ++i)
	{
		x[i] = Objectstacked_Centroid_and_Angle[i][0];
		y[i] = Objectstacked_Centroid_and_Angle[i][1];
		th[i] = Objectstacked_Centroid_and_Angle[i][2]+90;
		X[i] = Xcal1+Kx*(x[i]-xcal1);
		Y[i] = Ycal1+Ky*(y[i]-ycal1);

		char AboveObject[100];
		sprintf(AboveObject, "MOVL %f %f %f %f 0 180", X[i], Y[i], Zcal1+Hblock*i, th[i]);
		char OnObject[100];
		sprintf(OnObject, "MOVL %f %f %f %f 0 180", X[i], Y[i], Zcal1, th[i]);
		char AboveStackObject[100];
		sprintf(AboveStackObject, "MOVL %f %f %f %f 0 180", X[0], Y[0], Zcal1+Hblock*(i+1), th[0]);
		char OnStackObject[100];
		sprintf(OnStackObject, "MOVL %f %f %f %f 0 180", X[0], Y[0], Zcal1+Hblock*i, th[0]);

		// 4. Move the arm to the grasping pose by sendCommand() function.
		// The following lines give an example of how to send a command.
		// You can find commends in "Robot Arm Manual.pdf"
	
		// 5. Control the gripper to grasp the object.
		// The following lines give an example of how to control the gripper.
		//char closeGripper[] = "OUTPUT 48 ON";
		//sendCommand(closeGripper, ClientSocket);
		sendCommand(AboveObject, ClientSocket);
		sendCommand(OnObject, ClientSocket);
		sendCommand(closeGripper, ClientSocket);
		sendCommand(AboveObject, ClientSocket);
		sendCommand(AboveStackObject, ClientSocket);
		sendCommand(OnStackObject, ClientSocket);
		sendCommand(openGripper, ClientSocket);
		sendCommand(AboveStackObject, ClientSocket);
		
	}
	
	//Robot go home
	sendCommand(goHome, ClientSocket);
	
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