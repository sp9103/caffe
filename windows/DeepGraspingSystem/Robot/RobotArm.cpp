#include "RobotArm.h"


RobotArm::RobotArm(void)
{
}


RobotArm::~RobotArm(void)
{
}

int RobotArm::Init(int PortNum, int BaudRateNum, int *ID_list){
	SetID(ID_list);

	Com_port_num_ = PortNum;
	Baud_rate_num_ = BaudRateNum;

	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		return -1;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		return -1;
	}


	return 1;
}

int RobotArm::DeInit(){
	// Close port
	portHandler->closePort();

	return 1;
}

int RobotArm::SetID(int *ID_list){
	//각각 아이디 부여
	int i;
	for(i = 0; i < NUM_JOINT; i++)
		jointID_[i] = ID_list[i];
	for(int j = 0; j < NUM_FINGER; j++)
		fingerID_[j] = ID_list[i++];

	return 1;
}

int RobotArm::TorqueOn(){
	FingerTorqueOn();
	for (int i = 0; i < NUM_JOINT; i++){
		// Enable Dynamixel Torque
		int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, jointID_[i], PRO_TORQUE_ENABLE, 1, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	printf("=================Torque ON====================\n");
	return 1;
}

int RobotArm::TorqueOff(){
	FingerTorqueOff();
	for (int i = 0; i < NUM_JOINT; i++){
		// Enable Dynamixel Torque
		int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, jointID_[i], PRO_TORQUE_ENABLE, 0, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	printf("=================Torque OFF====================\n");
	return 1;
}

int RobotArm::isMoving(bool *ismove){
	isFingerMove(&ismove[NUM_JOINT]);
	for (int i = 0; i < NUM_JOINT; i++){
		uint8_t temp;
		int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, jointID_[i], PRO_MOVING, &temp, &dxl_error);
		ismove[i] = temp == 1 ? true : false;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	return 1;
}

int RobotArm::GetPresPosition(int *PresentPosition){
	GetJointPresPosition(PresentPosition);
	GetFingerPosition(&PresentPosition[NUM_JOINT]);

	return 1;
}

int RobotArm::GetGoalPosition(int *GoalPosition){
	GetJointGoalPosition(GoalPosition);
	for (int i = 0; i < NUM_FINGER; i++){
		uint16_t temp;
		int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, fingerID_[i], PRO_GOAL_POSITION, &temp, &dxl_error);
		GoalPosition[i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
	}

	return 1;
}

int RobotArm::GetPresVelocity(int *PresentVelocity){
	for (int i = 0; i < NUM_JOINT; i++){
		// Enable Dynamixel Torque
		uint32_t temp;
		int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, jointID_[i], PRO_PRESENT_VELOCITY, &temp, &dxl_error);
		PresentVelocity[i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}
	for (int i = 0; i < NUM_FINGER; i++){
		// Enable Dynamixel Torque
		uint16_t temp;
		int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, jointID_[i], MX_PRESENT_SPEED, &temp, &dxl_error);
		PresentVelocity[NUM_JOINT + i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	return 1;
}
int RobotArm::GetGoalVelocity(int *GoalVelocity){
	for (int i = 0; i < NUM_JOINT; i++){
		// Enable Dynamixel Torque
		uint32_t temp;
		int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, jointID_[i], PRO_GOAL_VELOCITY, &temp, &dxl_error);
		GoalVelocity[i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}
	for (int i = 0; i < NUM_FINGER; i++){
		// Enable Dynamixel Torque
		uint16_t temp;
		int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, jointID_[i], MX_MOVING_SPEED, &temp, &dxl_error);
		GoalVelocity[NUM_JOINT + i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	return 1;
}

int RobotArm::SetGoalVelocity(int *GoalVelocity){
	for (int i = 0; i < NUM_JOINT; i++){
		// Enable Dynamixel Torque
		uint32_t temp = (uint32_t)GoalVelocity[i];
		int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, jointID_[i], PRO_GOAL_VELOCITY, temp, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}
	for (int i = 0; i < NUM_FINGER; i++){
		// Enable Dynamixel Torque
		uint16_t temp = (uint16_t)GoalVelocity[i];
		int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, jointID_[i], MX_MOVING_SPEED, temp, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	return 1;
}

int RobotArm::SetGoalPosition(int *GoalPosition){
	SetJointPosition(GoalPosition);
	SetFingerPosition(&GoalPosition[NUM_JOINT]);

	return 1;
}

int RobotArm::GetFingerLoad(int *load){
	for (int i = 0; i < 3; i++){
		uint16_t temp;
		int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, fingerID_[i], MX_PRESENT_LOAD, &temp, &dxl_error);
		load[i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	return 1;
}

int RobotArm::SetFingerPosition(int *GoalPosition){
	for (int i = 0; i < NUM_FINGER; i++){
		// Enable Dynamixel Torque
		uint16_t temp = (uint16_t)GoalPosition[i];
		int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, fingerID_[i], MX_GOAL_POSITION, temp, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	return 1;
}

int RobotArm::SetJointPosition(int *GoalPosition){
	for (int i = 0; i < NUM_JOINT; i++){
		// Enable Dynamixel Torque
		uint32_t temp = (uint32_t)GoalPosition[i];
		int dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, jointID_[i], PRO_GOAL_POSITION, temp, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}

	return 1;
}

int RobotArm::Arm_Get_JointValue(Eigen::VectorXi *value)
{
	value->resize(NUM_JOINT);

	int temp_val[9];
	GetPresPosition(temp_val);

	for(int i = 0; i < NUM_JOINT; i++){
		(*value)[i] = temp_val[i];
	}

	return 1;
}

void RobotArm::safeReleasePose(){
	int ReleasePose[] = {0,0,0,0,0,0};
	int presPose[6];
	int goalPose[6];

	GetJointPresPosition(presPose);
	memcpy(goalPose, presPose, sizeof(int) * 6);
	goalPose[1] = ReleasePose[1];
	SetJointPosition(goalPose);
	waitMove();
	goalPose[3] = ReleasePose[3];
	SetJointPosition(goalPose);
	waitMove();
	SetJointPosition(ReleasePose);
	waitMove();
}

bool RobotArm::waitMove(){
	int presPose[6], goalPose[6];

	while(1){
		GetJointPresPosition(presPose);
		GetJointGoalPosition(goalPose);
		int max = -9999;
		for(int i = 0; i < 6; i++){
			int sub = abs(presPose[i] - goalPose[i]);
			if(sub > max)	max = sub;
		}
		if(max < 30)	break;
		Sleep(10);
	}

	return true;
}

void RobotArm::safeMovePose(int *goal){
	int presPose[6];

	GetJointPresPosition(presPose);
	presPose[0] = goal[0];
	presPose[2] = goal[2];
	presPose[4] = goal[4];
	presPose[5] = goal[5];
	SetJointPosition(presPose);
	waitMove();
	presPose[3] = goal[3];
	SetJointPosition(presPose);
	waitMove();
	SetJointPosition(goal);
	waitMove();
}

int RobotArm::GetJointPresPosition(int *PresentPosition){
	for (int i = 0; i < NUM_JOINT; i++){
		// Enable Dynamixel Torque
		uint32_t temp;
		int dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, jointID_[i], PRO_PRESENT_POSITION, &temp, &dxl_error);
		PresentPosition[i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return -1;
		}
	}
}

void RobotArm::FingerTorqueOn(){
	for (int i = 0; i < NUM_FINGER; i++){
		// Enable Dynamixel Torque
		int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, fingerID_[i], MX_TORQUE_ENABLE, 1, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
	}
}

void RobotArm::FingerTorqueOff(){
	for (int i = 0; i < NUM_FINGER; i++){
		// Enable Dynamixel Torque
		int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, fingerID_[i], MX_TORQUE_ENABLE, 0, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
	}
}

int RobotArm::GetFingerPosition(int *presntPosition){
	for (int i = 0; i < 3; i++){
		uint16_t temp;
		int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, fingerID_[i], MX_PRESENT_POSITION, &temp, &dxl_error);
		presntPosition[i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
	}

	return 1;
}

int RobotArm::GetJointGoalPosition(int *GoalPosition){
	for (int i = 0; i < NUM_JOINT; i++){
		uint16_t temp;
		int dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, jointID_[i], PRO_GOAL_POSITION, &temp, &dxl_error);
		GoalPosition[i] = (int)temp;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
	}
	return 1;
}

int RobotArm::isFingerMove(bool *ismove){
	for (int i = 0; i < 3; i++){
		uint8_t temp;
		int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, fingerID_[i], MX_MOVING, &temp, &dxl_error);
		ismove[i] = temp == 1 ? true : false;
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
		}
	}

	return 1;
}