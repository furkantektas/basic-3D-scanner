#include "motordriver.h"

int MotorDriver::stepCount = 0;

MotorDriver::MotorDriver() : arduinoPort(0), baudrate(115200) {
    // int arduinoPort = 0; //  /dev/ttyACMA
    if ((port = OpenComport(arduinoPort, baudrate)))
        throw UnableToConnectMotor();
    MotorDriver::stepCount = 0;
}

MotorDriver::~MotorDriver() {
    CloseComport(arduinoPort);
}

void MotorDriver::stepLeft(int stepCount) {
    step(stepCount, Left);
    MotorDriver::setStepCount(MotorDriver::getStepCount() - stepCount);
}

void MotorDriver::stepRight(int stepCount) {
    step(stepCount, Right);
    MotorDriver::setStepCount(MotorDriver::getStepCount() + stepCount);
}

void MotorDriver::step(int stepCount, Direction d) {
    for (int i = 0; i < stepCount; ++i) {
        if (SendByte(arduinoPort, (d ? '1' : '0')))
            throw UnableToConnectMotor();
        #ifdef _WIN32
        Sleep(50);
        #elif __linux
        usleep(50000);
        #endif  
    }
}
/*

int main(void) {
	MotorDriver m;
	m.stepLeft(50);
	std::cout << "Step Count " << MotorDriver::getStepCount() << std::endl;
	usleep(50000);
	m.stepRight(30);
	std::cout << "Step Count " << MotorDriver::getStepCount() << std::endl;
}*/