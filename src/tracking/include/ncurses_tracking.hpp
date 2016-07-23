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
char commandstring[] = "[0]toggle pose publishing, [1]show virtual marker, [9]exit";
char invalidstring[] = "invalid!";
char quitstring[] = " [hit q to quit]";
char averageconnectionspeedstring[] = "average connection speed: ";
char logfilestring[] = "see logfile measureConnectionTime.log for details";
char filenamestring[] = "enter filename to save recorded trajectories: ";
char byebyestring[] = "BYE BYE!";

class NCurses_tracking{
public:
    NCurses_tracking(){
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

        print(0,0,cols,"-");
        printMessage(1,0,welcomestring);
        print(2,0,cols,"-");
        print(6,0,cols,"-");
//		querySensoryData();
        printMessage(3,0,commandstring);
	}
	~NCurses_tracking(){
        delete model;
		clearAll(0);
		printMessage(rows/2,cols/2-strlen(byebyestring)/2,byebyestring);
		refresh();
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
		timeout(-1);
		echo();
		print(4,0,cols," ");
		print(5,0,cols," ");

		print(4,0,cols," ");
		print(5,0,cols," ");
		noecho();
	}
    void toggleVirtualMarker(){
        timeout(-1);
        echo();
        print(4,0,cols," ");
        print(5,0,cols," ");

        print(4,0,cols," ");
        print(5,0,cols," ");
        noecho();
    }
    Model *model;
    MarkerTracker markerTracker;
private:
	uint rows, cols;
	float pos;
	char inputstring[30];
};
