#pragma once

#include "markertracker.hpp"
#include "model.hpp"
#include <ncurses.h>

enum COLORS{
	CYAN=1,
	RED,
	GREEN,
};

//! standard query messages
char welcomestring[] = "commandline tool for controlling raspberry pi motion capture system";
char commandstring[] = "[0]toggle pose publishing, [1]show virtual marker, [2]stream video, [9]exit";
char posepublishingstring[] = "pose publishing ";
char virtualmarkerstring[] = "virtual marker rendering ";
char streamvideostring[] = "stream video ";
char onstring[] = "ON";
char offstring[] = "OFF";
char fpsstring[] =    "fps:          ";
char markerstring[] = "marker:       ";
char invalidstring[] = "invalid!";
char quitstring[] = " [hit q to quit]";
char logfilestring[] = "see logfile measureConnectionTime.log for details";
char filenamestring[] = "enter filename to save recorded trajectories: ";
char byebyestring[] = "BYE BYE!";

class NCurses_tracking{
public:
    NCurses_tracking(sf::Window *win):window(win){
        model = new Model("/home/letrend/workspace/markertracker","markermodel.dae");
        Vector3f cameraPosition(0,0,0);
        Vector3f point(0,0,1);
        // first person camera
        model->lookAt(point,cameraPosition);

        //! start ncurses mode
        initscr();
        //! Start color functionality
        start_color();
        init_pair(CYAN, COLOR_CYAN, COLOR_BLACK);
        init_pair(RED, COLOR_RED, COLOR_BLACK);
        init_pair(GREEN, COLOR_GREEN, COLOR_BLACK);
        //! get the size of the terminal window
        getmaxyx(stdscr,rows,cols);

        // how many camera infos can we render in a row
        uint camerasPerRow = cols/30;

        print(0,0,cols,"-");
        printMessage(1,0,welcomestring);
        print(2,0,cols,"-");
        print(6,0,cols,"-");
        printMessage(3,0,commandstring);

        window->setActive(virtualMarkerVisibleFlag);
        window->setVisible(virtualMarkerVisibleFlag);
	}
	~NCurses_tracking(){
        delete model;
		clearAll(0);
		printMessage(rows/2,cols/2-strlen(byebyestring)/2,byebyestring);
		usleep(1000000);
		endwin();
	}
	void printMessage(uint row, uint col, char* msg){
		mvprintw(row,col,"%s", msg);
		refresh();
	}
	void printMessage(uint row, uint col, char* msg, uint color){
		mvprintw(row,col,"%s", msg);
		mvchgat(row, col, strlen(msg), A_BOLD, color, NULL);
		refresh();
	}
	void print(uint row, uint startcol, uint length, const char* s){
		for (uint i=startcol;i<startcol+length;i++){
			mvprintw(row,i,"%s",s);
		}
		refresh();
	}
	void clearAll(uint row){
		for(uint i=row;i<rows;i++){
			print(i,0,cols," ");
		}
		refresh();
	}
	void processing(char* msg1, char* what, char* msg2){
		char cmd;
		uint a = strlen(msg1);
		uint b = strlen(what);
		uint c = strlen(msg2);

		print(5,0,cols," ");
		printMessage(5,0,msg1);
		printMessage(5,a+1, what);
		printMessage(5,a+1+b+1, msg2);
		mvchgat(5, 0,       a+1+b, A_BLINK, 2, NULL);
		mvchgat(5, a+1+b+1, a+1+b+1+c, A_BLINK, 1, NULL);
		timeout(10);
		do{
//			querySensoryData();
			cmd = mvgetch(5,a+1+b+1+c);
		}while(cmd != 'q');
		timeout(-1);
	}
	void processing(char* msg1, char* msg2){
		char cmd;
		uint a = strlen(msg1);
		uint c = strlen(msg2);

		print(5,0,cols," ");
		printMessage(5,0,msg1);
		printMessage(5,a+1, msg2);
		mvchgat(5, 0,       a, A_BLINK, 2, NULL);
		mvchgat(5, a+1, a+1+c, A_BLINK, 1, NULL);
		timeout(10);
		do{
//			querySensoryData();
			cmd = mvgetch(5,a+1+c);
		}while(cmd != 'q');
		timeout(-1);
	}
	void togglePosePublishing(){
        pubishPoseFlag = !pubishPoseFlag;
        if(pubishPoseFlag){
            markerTracker.rviz_marker_pub = markerTracker.nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
        }else{
            markerTracker.rviz_marker_pub.shutdown();
        }
        print(4,0,cols," ");
        printMessage(4,0,posepublishingstring);
        printMessage(4,strlen(posepublishingstring),pubishPoseFlag ? onstring:offstring, GREEN);
        usleep(500000);
        print(4,0,cols," ");
	}
    void toggleVirtualMarker(){
        virtualMarkerVisibleFlag = !virtualMarkerVisibleFlag;
        if(virtualMarkerVisibleFlag) {
            window->setActive(virtualMarkerVisibleFlag);
            window->setVisible(virtualMarkerVisibleFlag);
            // update viewmatrix from user input
            model->updateViewMatrix(*window);
            // render the object with the updated modelmatrix
            pose = markerTracker.ModelMatrix.cast<float>();
            model->render(pose, img);
            window->display();
        }
        print(4,0,cols," ");
        printMessage(4,0,virtualmarkerstring);
        printMessage(4,strlen(virtualmarkerstring),virtualMarkerVisibleFlag ? onstring:offstring, GREEN);
        usleep(500000);
        print(4,0,cols," ");
    }
    void streamVideo(){
        streamVideoFlag = !streamVideoFlag;
        markerTracker.sendCameraControl(0,0,streamVideoFlag);
        print(4,0,cols," ");
        printMessage(4,0,streamvideostring);
        printMessage(4,strlen(streamvideostring),streamVideoFlag ? onstring:offstring, GREEN);
        usleep(500000);
        print(4,0,cols," ");
    }
    void showCameraInfo(){
        uint cam = 0, row = 6;
        for(auto state = markerTracker.cameraState.begin(); state != markerTracker.cameraState.end(); ++state) {
            printMessage(6, cam*30, markerTracker.camera[state->first].name);
            switch(state->second){
                case Uninitialized:
                    printMessage(row+1, cam*30, (char*)"Unintialized", COLOR_WHITE);
                    break;
                case Initialized:
                    printMessage(row+1, cam*30, (char*)"Intialized", COLOR_YELLOW);
                    break;
                case Tracking:
                    printMessage(row+1, cam*30, (char*)"Tracking", COLOR_GREEN);
                    break;
                case Error:
                    printMessage(row+1, cam*30, (char*)"Error", COLOR_RED);
                    break;
            }
            printMessage(row+2, cam*30, markerTracker.camera[state->first].name);
            sprintf(fpsstring,"fps %f", markerTracker.camera[state->first].fps);
            printMessage(row+3, cam*30, fpsstring);
            sprintf(markerstring, "marker %d", markerTracker.camera[state->first].markerVisible);
            printMessage(row+4, cam*30, markerstring);
            cam++;
            if(row>camerasPerRow) {
                row += 6;
                cam = 0;
            }
        }
        if(streamVideoFlag){
            if(!markerTracker.lockWhileWriting) {
                cv::imshow("video stream", markerTracker.cv_ptr->image);
                cv::waitKey(1);
            }
        }
    }
private:
    Model *model;
    MarkerTracker markerTracker;
    sf::Window *window;
	uint rows, cols, camerasPerRow;
    bool virtualMarkerVisibleFlag = false, pubishPoseFlag = true, streamVideoFlag = false;
	float pos;
	char inputstring[30];
    Mat img;
    Matrix4f pose;
};
