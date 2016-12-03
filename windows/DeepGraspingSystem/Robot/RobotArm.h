#pragma once
#include <stdio.h>
#include <Eigen\Dense>
#include <windows.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

#pragma region DynamixelDefine

#define MAX_MOVING_DEGREE					10
#define USING_FINGER

//MX FINGER
#define NUM_FINGER                        3

#define MX_CW_ANGLE_LIMIT                6
#define MX_CCW_ANGLE_LIMIT                8

#define MX_TORQUE_ENABLE                24
#define MX_LED_ONOFF                    25
#define MX_D_GAIN                        26
#define MX_I_GAIN                        27
#define MX_P_GAIN                        28
#define MX_GOAL_POSITION                30
#define MX_MOVING_SPEED                    32
#define MX_TORQUE_LIMIT                    34
#define MX_PRESENT_POSITION                36
#define MX_PRESENT_SPEED                38
#define MX_PRESENT_LOAD                    40
#define MX_PRESENT_TEMPERATURE            43 //Address of Present Temperature in Control Table
#define MX_GOAL_ACCELERATION            77
#define MX_MOVING                        46

//PRO Manipulator
#define NUM_JOINT                            6
#define PRO_OPERATING_MODE            11
#define PRO_TORQUE_ENABLE            562 //Address of Torque Enable in Control Table
#define PRO_LED_RED                    563
#define PRO_LED_GREEN                564
#define PRO_LED_BLUE                565

//Not Used - Ignore this address
#define PRO_VELOCITY_I_GAIN            586
#define PRO_VELOCITY_P_GAIN            588
#define    PRO_POSITION_P_GAIN            594

#define PRO_GOAL_POSITION            596 //Address of Goal Position in Control Table
#define PRO_GOAL_VELOCITY            600
#define PRO_GOAL_TORQUE                604
#define PRO_GOAL_ACCELERATION        606

//Read ONLY
#define PRO_MOVING                    610
#define PRO_PRESENT_POSITION        611 //Address of Goal Position in Control Table
#define PRO_PRESENT_VELOCITY        615
#define PRO_PRESENT_LOAD            619
#define PRO_PRESENT_CURRENT            621 //Address of Present Current in Control Table
#define PRO_PRESENT_INPUT_VOLTAGE    623
#define PRO_PRESENT_TEMPERATURE        625 //Address of Present Temperature in Control Table

//Mode
#define DYNAMIXELPRO_JOINTMODE        3
#define DYNAMIXELPRO_WHEELMODE        1
#define DYNAMIXELPRO_TORQUEMODE        0

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "COM5"      // Check which port is being used on your controller
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#pragma endregion


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

	int GetPresPosition(int *PresentPosition = NULL);					//현재 위치 받아오기
	int GetGoalPosition(int *GoalPosition = NULL);						//목표 위치 받아오기
	int isMoving(bool *ismove = NULL);								//움직임 유무 받아오기
	int GetPresVelocity(int *PresentVelocity = NULL);					//현재 속도 받아오기
	int GetGoalVelocity(int *GoalVelocity = NULL);						//목표 속도 받아오기

	int SetGoalVelocity(int *GoalVelocity);
	int SetGoalPosition(int *GoalPosition);
	//Arm
	int SetJointPosition(int *GoalPosition);
	int GetJointGoalPosition(int *GoalPosition = NULL);
	int GetJointPresPosition(int *PresentPosition);

	//Finger
	void FingerTorqueOn();
	void FingerTorqueOff();
	int GetFingerLoad(int *load = NULL);
	int GetFingerPosition(int *presntPosition);
	int isFingerMove(bool *ismove);
	int SetFingerPosition(int *GoalPosition);

	void safeReleasePose();
	void safeMovePose(int *goal);

	//ARMSDK function
	int Arm_Get_JointValue(Eigen::VectorXi *value);

	bool waitMove();

private:
	int fingerID_[NUM_FINGER];
	int jointID_[NUM_JOINT];
	int Com_port_num_;
	int Baud_rate_num_;
	uint8_t dxl_error = 0;

	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;

};

