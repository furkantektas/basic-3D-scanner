#ifndef MOTORDRIVER_H
#define	MOTORDRIVER_H


#include "rs232.h"

#ifdef _WIN32
#include <windows.h>
#elif __linux
#include <unistd.h> // usleep
#endif  

#include <exception>
#include <stdexcept>
#include <iostream>


struct UnableToConnectMotor : public std::exception {
	const char * what () const throw () {
		return "Unable to connect motor driver!";
	}
};

class MotorDriver{
	public:
	typedef enum {Left=0, Right} Direction;
	
	MotorDriver();

	~MotorDriver();

	void stepLeft(int stepCount = 1);

	void stepRight(int stepCount = 1);

	static inline int getStepCount() { return MotorDriver::stepCount; }
	static inline void setStepCount(int count) { MotorDriver::stepCount = count; }

	private:
		int port;
		int arduinoPort;
		int baudrate;
		static int stepCount; // Right -> + || Left -> -
	
	void step(int stepCount, Direction d);

};

//int MotorDriver::stepCount = 0;

/*
int main(void) {
	MotorDriver m;
	m.stepLeft(50);
	std::cout << "Step Count " << MotorDriver::getStepCount() << std::endl;
	usleep(50000);
	m.stepRight(30);
	std::cout << "Step Count " << MotorDriver::getStepCount() << std::endl;
}*/

#endif