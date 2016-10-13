#pragma once
#include <stdio.h>
#include <Eigen\Dense>
#include <windows.h>

#include "dynamixel.h"
#include "MX_controller.h"
#include "DynamixelPro.h"

#define MAX_MOVING_DEGREE					10
#define USING_FINGER

class RobotArm
{
public:
	RobotArm(void);
	~RobotArm(void);

	int Init(int PortNum, int BaudRateNum, int *ID_list);
	int DeInit();
	int SetID(int *ID_list);

	int TorqueOn();
	int TorqueOff();
	void FingerTorqueOn();
	void FingerTorqueOff();

	int GetTemperature(int *Temperature = NULL);						//�µ� �޾ƿ���
	int GetPresPosition(int *PresentPosition = NULL);					//���� ��ġ �޾ƿ���
	int GetGoalPosition(int *GoalPosition = NULL);						//��ǥ ��ġ �޾ƿ���
	int isMoving(bool *ismove = NULL);								//������ ���� �޾ƿ���
	int GetPresVelocity(int *PresentVelocity = NULL);					//���� �ӵ� �޾ƿ���
	int GetGoalVelocity(int *GoalVelocity = NULL);						//��ǥ �ӵ� �޾ƿ���
	int GetFingerLoad(int *load = NULL);
	int GetFingerPosition(int *presntPosition);
	int isFingerMove(bool *ismove);
	int GetJointGoalPosition(int *GoalPosition = NULL);

	int SetGoalVelocity(int *GoalVelocity);
	int SetGoalPosition(int *GoalPosition);
	int SetLED(bool onoff);
	int SetFingerPosition(int *GoalPosition);
	int SetJointPosition(int *GoalPosition);

	void safeReleasePose();
	void safeMovePose(int *goal);

	//ARMSDK function
	int Arm_Get_JointValue(Eigen::VectorXi *value);
	SerialPort* DXL_Get_Port(void);

	bool waitMove();

private:
	int fingerID_[NUM_FINGER];
	int jointID_[NUM_JOINT];
	int Com_port_num_;
	int Baud_rate_num_;

	SerialPort sp_;
	SerialPort *Port_;

	DynamixelPro jointcontroller_;
#ifdef USING_FINGER
	MX_controller fingercontroller_;
#endif
};

