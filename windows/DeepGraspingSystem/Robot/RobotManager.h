#pragma once
#include "..\ARMSDK\include\ARMSDK.h"
#include "..\stdafx.h"
#include "RobotArm.h"

#ifdef _DEBUG
#pragma comment(lib, "ARMSDKd.lib")
#endif
#ifdef NDEBUG
#pragma comment(lib, "ARMSDK.lib") 
#endif

class RobotManager
{
public:
	RobotManager();
	~RobotManager();

	int Initialize(int PortNum, int BaudRateNum);
	int DeInitialize();

	void Approaching(int *pos);
	void Lift();

	void safeRelease();
	void safeMove(int *pos);

	void TorqueOn();
	void TorqueOff();

	void setVel(int* src);
	void Move(int *pos);
	
	void getPresState(int *dst);
	
	//finger
	void FingerTorqueOn();
	void FingerTorqueOff();
	void grasp();

private:
	int FinLimitMotion[2][3];

	armsdk::RobotInfo robot;
	armsdk::Kinematics kin;
	RobotArm arm;

	void ControllerInit(int PortNum, int BaudRateNum);
	bool robotConnectCheck();

	void FingerLimit(int *src);
};

