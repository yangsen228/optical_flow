#ifndef GET_FPS_H
#define GET_FPS_H

#include <time.h>
#include <iostream>

using namespace std;

class GetFps{
public:
	int current_time_;
	int previous_time_;
	int fps_;

	GetFps(){
		current_time_ = 0;
		previous_time_ = 0;
		fps_ = 0;
	}

	void getFps(int &fps){
		current_time_ = time(0);
		if (previous_time_ == current_time_){
			fps_ ++;
			previous_time_ = current_time_;
		}
		else{
			fps = fps_;
			fps_ = 0;
			previous_time_ = current_time_;
		}	
	}
};

#endif
