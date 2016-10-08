#include "RobotManager.h"


RobotManager::RobotManager()
{
	FinLimitMotion[0][0] = 2973;
	FinLimitMotion[1][0] = 2533;
	FinLimitMotion[0][1] = 1527;
	FinLimitMotion[1][1] = 1097;
	FinLimitMotion[0][2] = 2052;
	FinLimitMotion[1][2] = 1774;
}


RobotManager::~RobotManager()
{
}


void RobotManager::ControllerInit(int PortNum, int BaudRateNum){
	int robotid[] = { 1, 3, 5, 7, 9, 11, 13, 15, 17 };
	int vel[] = { 1000, 1000, 1000, 1000, 1000, 1000, 50, 50, 50 };
	arm.Init(PortNum, BaudRateNum, robotid);
	arm.SetGoalVelocity(vel);
}

bool RobotManager::robotConnectCheck(){
	veci angi(6);
	arm.Arm_Get_JointValue(&angi);

#ifdef RIGHT_ARM_USE
	//RightArm
	robot.AddJoint(0.0, ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 1);
	robot.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 3);
	robot.AddJoint(30.0, -ML_PI_2, 246.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 5);
	robot.AddJoint(-30.0, ML_PI_2, 0.0, ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 7);
	robot.AddJoint(0.0, -ML_PI_2, 216.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 9);
	robot.AddJoint(0.0, ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 11);
#elif defined LEFT_ARM_USE
	//Leftarm
	robot.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 2);
	robot.AddJoint(0.0, ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 4);
	robot.AddJoint(30.0, ML_PI_2, 246.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 6);
	robot.AddJoint(-30.0, -ML_PI_2, 0.0, -ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 8);
	robot.AddJoint(0.0, ML_PI_2, 216.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 10);
	robot.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 12);
#endif
	kin.InitRobot(&robot);

	//맥시멈 앵글 체크 - 쓰레기값 걸러내기
	for (int JointNum = 0; JointNum < 6; JointNum++)
	{
		if (abs(angi[JointNum]) > robot.GetJointInfo(JointNum)->GetMaxAngleInValue() + 10)
		{
			printf("[%d] Data Fail %d\n", JointNum, angi[JointNum]);
		}
	}

	return true;
}

int RobotManager::Initialize(int PortNum, int BaudRateNum){
	ControllerInit(PortNum, BaudRateNum);
	if (!robotConnectCheck()){
		printf("Robot Init fail\n");
		return -1;
	}
}

int RobotManager::DeInitialize(){
	arm.DeInit();

	return 0;
}

void RobotManager::safeRelease(){
	arm.safeReleasePose();
}

void RobotManager::safeMove(int *pos){
	arm.safeMovePose(pos);
}

void RobotManager::TorqueOn(){
	arm.TorqueOn();
}

void RobotManager::TorqueOff(){
	arm.TorqueOff();
}

void RobotManager::Move(int *pos){
	int presPos[NUM_XEL];
	arm.GetGoalPosition(presPos);

	while (1){
		int goalPos[NUM_XEL];
		arm.SetGoalPosition(pos);
		arm.GetGoalPosition(goalPos);

		int i = 0;
		for (i = 0; i < NUM_XEL; i++){
			if (pos[i] != goalPos[i]){
				break;
			}
		}
		if (i == NUM_XEL)
			break;
	}

	//움직일때까지 기다리는 부분
	arm.waitMove();
}

void RobotManager::FingerLimit(int *src){
	for (int i = 0; i < 3; i++){
		int min = FinLimitMotion[1][i];
		int max = FinLimitMotion[0][i];
		if (src[i] > max)
			src[i] = max;
		if (src[i] < min)
			src[i] = min;
	}
}

void RobotManager::Approaching(int *pos){
	FingerLimit(&pos[NUM_JOINT]);

	int presPose[6];
	if (pos[1] > 151471)	pos[1] = 151471;
	if (pos[1] < -152820)	pos[1] = -152820;
	
	arm.SetFingerPosition(&pos[NUM_JOINT]);
	arm.safeMovePose(pos);
}

void RobotManager::FingerTorqueOn(){
	arm.FingerTorqueOn();
}

void RobotManager::FingerTorqueOff(){
	arm.FingerTorqueOff();
}

void RobotManager::setVel(int* src){
	arm.SetGoalVelocity(src);
}

void RobotManager::getPresState(int *dst){
	arm.GetPresPosition(dst);
}

void RobotManager::grasp(){
	int State[NUM_FINGER];
	arm.SetFingerPosition(FinLimitMotion[1]);

	while (1){
		arm.GetFingerPosition(State);
		//LoadCheck
	}
}