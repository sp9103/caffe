#pragma once
#include <stdio.h>
#include <stdlib.h>

#include "dynamixel.h"

#define NUM_FINGER						3

#define MX_CW_ANGLE_LIMIT				6
#define MX_CCW_ANGLE_LIMIT				8

#define MX_TORQUE_ENABLE				24
#define MX_LED_ONOFF					25
#define MX_D_GAIN						26
#define MX_I_GAIN						27
#define MX_P_GAIN						28
#define MX_GOAL_POSITION				30
#define MX_MOVING_SPEED					32
#define MX_TORQUE_LIMIT					34
#define MX_PRESENT_POSITION				36
#define MX_PRESENT_SPEED				38
#define MX_PRESENT_LOAD					40
#define MX_PRESENT_TEMPERATURE			43 //Address of Present Temperature in Control Table
#define MX_GOAL_ACCELERATION			77
#define MX_MOVING						46

class MX_controller
{
public:
	MX_controller(void);
	~MX_controller(void);

	void Init(SerialPort port, int *id);

	int TorqueOn();
	int TorqueOff();

	int GetTemperature(int *Temperature = NULL);		//�µ� �޾ƿ���
	int GetPresPosition(int *PresentPosition = NULL);					//���� ��ġ �޾ƿ���
	int GetGoalPosition(int *GoalPosition = NULL);						//��ǥ ��ġ �޾ƿ���
	int isMoving(bool *ismove = NULL);					//������ ���� �޾ƿ���
	int GetPresVelocity(int *PresentVelocity = NULL);					//���� �ӵ� �޾ƿ���
	int GetGoalVelocity(int *GoalVelocity = NULL);						//��ǥ �ӵ� �޾ƿ���
	int GetPresLoad(int *PresentLoad = NULL);

	int SetGoalVelocity(int *GoalVelocity);
	int SetGoalPosition(int *GoalPosition);
	int SetLED(unsigned char *LED);

private:
	int ID_list_[NUM_FINGER];
	unsigned char mode_;

	int DataValidCheck(int *src, int Max, int Min);

	BulkData bd[256];
	BulkData *pbd[256];

	SerialPort sp_;
	SerialPort *Port_;
};

