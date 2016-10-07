#include "DynamixelPro.h"


DynamixelPro::DynamixelPro(void)
{
	sp_.dPacketStartTime = 0;
	sp_.fByteTransferTime = 0;
	sp_.fPacketWaitTime = 0;
	sp_.hComm = 0;
	sp_.iBusUsing = 0;

	mode_ = -1;

	Port_ = &sp_;

	for(unsigned int i = 0 ; i <256 ; i++)
	{
		pbd[i] = &bd[i];
	}
}


DynamixelPro::~DynamixelPro(void)
{
}

void DynamixelPro::Init(SerialPort port, int *id){
	for(int i = 0; i < NUM_JOINT; i++)
		ID_list_[i] = id[i];

	sp_ = port;
}

int DynamixelPro::TorqueOn(){
	int ErrorStatus;
	for(int i = 0; i < NUM_JOINT; i++){
		int Result = dxl_write_byte(Port_, ID_list_[i], PRO_TORQUE_ENABLE, 1, &ErrorStatus);

		if( Result != COMM_RXSUCCESS )
		{
			printf( "Failed DynamixelPro Enable!\n" );
			return -1;
		}
	}

	return 1;
}

int DynamixelPro::TorqueOff(){
	int ErrorStatus;
	for(int i = 0; i < NUM_JOINT; i++){
		int Result = dxl_write_byte(Port_, ID_list_[i], PRO_TORQUE_ENABLE, 0, &ErrorStatus);

		if( Result != COMM_RXSUCCESS )
		{
			printf( "Failed DynamixelPro Enable!\n" );
			return -1;
		}
	}

	return 1;
}

int DynamixelPro::GetTemperature(int *Temperature){
	int tP_temperature[NUM_JOINT], temperature_length = 1;
	unsigned char *t_param;
	t_param = (unsigned char*)malloc(sizeof(unsigned char)*(NUM_JOINT*5));

	for(int i = 0; i < NUM_JOINT; i++){
		t_param[i*5] = (unsigned char)ID_list_[i];
		t_param[i*5+1] = DXL_LOBYTE(PRO_PRESENT_TEMPERATURE);
		t_param[i*5+2] = DXL_HIBYTE(PRO_PRESENT_TEMPERATURE);
		t_param[i*5+3] = DXL_LOBYTE(temperature_length);
		t_param[i*5+4] = DXL_HIBYTE(temperature_length);
	}

	dxl_bulk_read(Port_, t_param, NUM_JOINT*5, pbd);

	for(int i = 0; i < NUM_JOINT; i++){
		dxl_get_bulk_byte(pbd, ID_list_[i], PRO_PRESENT_TEMPERATURE, &tP_temperature[i]);
		printf("ID %d : %d\n", ID_list_[i], tP_temperature[i]);

		if(Temperature != NULL){
			Temperature[i] = tP_temperature[i];
		}
	}

	free(t_param);

	return 1;
}

int DynamixelPro::isMoving(bool *ismove){
	int tP_position[NUM_JOINT];

	int ErrorState;
	for(int i = 0; i < NUM_JOINT; i++){
		dxl_read_byte(Port_, ID_list_[i], PRO_PRESENT_POSITION, &tP_position[i], &ErrorState);

		printf("ID %d : %d\n", ID_list_[i], tP_position[i]);
		if(ismove != NULL){
			ismove[i] = tP_position[i] == 1 ? true : false;
		}
	}

	return 1;
}

int DynamixelPro::GetPresPosition(int *PresentPosition){
	int tP_position[NUM_JOINT];

	int ErrorState;
	for(int i = 0; i < NUM_JOINT; i++){
		dxl_read_dword(Port_, ID_list_[i], PRO_PRESENT_POSITION, (unsigned*)&tP_position[i], &ErrorState);

		if(PresentPosition != NULL){
			PresentPosition[i] = tP_position[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_position[i]);
	}

	return 1;
}

int DynamixelPro::GetGoalPosition(int *GoalPosition){
	int tG_position[NUM_JOINT];

	int ErrorState;
	for(int i = 0; i < NUM_JOINT; i++){
		dxl_read_dword(Port_, ID_list_[i], PRO_GOAL_POSITION, (unsigned*)&tG_position[i], &ErrorState);

		if(GoalPosition != NULL){
			GoalPosition[i] = tG_position[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tG_position[i]);
	}

	return 1;
}

int DynamixelPro::GetPresVelocity(int *PresentVelocity){
	int tP_velocity[NUM_JOINT];

	int ErrorState;
	for(int i = 0; i < NUM_JOINT; i++){
		dxl_read_dword(Port_, ID_list_[i], PRO_PRESENT_VELOCITY, (unsigned*)&tP_velocity[i], &ErrorState);

		if(PresentVelocity != NULL){
			PresentVelocity[i] = tP_velocity[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tP_velocity[i]);
	}

	return 1;
}

int DynamixelPro::GetGoalVelocity(int *GoalVelocity){
	int tG_velocity[NUM_JOINT];

	int ErrorState;
	for(int i = 0; i < NUM_JOINT; i++){
		dxl_read_dword(Port_, ID_list_[i], PRO_GOAL_VELOCITY, (unsigned*)&tG_velocity[i], &ErrorState);

		if(GoalVelocity != NULL){
			GoalVelocity[i] = tG_velocity[i];
		}else
			printf("ID %d : %d\n", ID_list_[i], tG_velocity[i]);
	}

	return 1;
}

int DynamixelPro::SetGoalPosition(int *GoalPosition){
	int ErrorStatus;
	for(int i = 0; i < NUM_JOINT; i++){
		int Result = dxl_write_dword(Port_, ID_list_[i], PRO_GOAL_POSITION, GoalPosition[i], &ErrorStatus);

		if( Result != COMM_RXSUCCESS )
		{
			printf( "Failed to set goal position!\n" );
			return -1;
		}
	}

	return 1;
}

int DynamixelPro::SetGoalVelocity(int *GoalVelocity){
	int ErrorStatus;
	for(int i = 0; i < NUM_JOINT; i++){
		int Result = dxl_write_dword(Port_, ID_list_[i], PRO_GOAL_VELOCITY, GoalVelocity[i], &ErrorStatus);

		if( Result != COMM_RXSUCCESS )
		{
			printf( "Failed to set goal Velocity!\n" );
			return -1;
		}
	}

	return 1;
}

//RED LED
int DynamixelPro::SetLED(unsigned char *LED){
	int ErrorStatus;
	for(int i = 0; i < NUM_JOINT; i++){
		int Result = dxl_write_byte(Port_, ID_list_[i], PRO_LED_RED, LED[i], &ErrorStatus);

		if( Result != COMM_RXSUCCESS )
		{
			printf( "Failed to change red LED!\n" );
			return -1;
		}
	}
	return 1;

}

int DynamixelPro::GetPresLoad(int *PresentLoad){
	int tP_load[NUM_JOINT];

	int ErrorState;
	for(int i = 0; i < NUM_JOINT; i++){
		dxl_read_word(Port_, ID_list_[i], PRO_PRESENT_LOAD, &tP_load[i], &ErrorState);

		if(PresentLoad != NULL){
			PresentLoad[i] = tP_load[i];
		}else{
			printf("ID %d : %d\n", ID_list_[i], (short)tP_load[i]);
		}
	}

	return 1;
}

int DynamixelPro::GetPresCurrent(int *PresentCurrent){
	int tP_current[NUM_JOINT];

	int ErrorState;
	for(int i = 0; i < NUM_JOINT; i++){
		dxl_read_word(Port_, ID_list_[i], PRO_PRESENT_CURRENT, &tP_current[i], &ErrorState);

		if(PresentCurrent != NULL){
			PresentCurrent[i] = tP_current[i];
		}else{
			printf("ID %d : %d\n", ID_list_[i], (short)tP_current[i]);
		}
	}

	printf("\n");

	return 1;
}