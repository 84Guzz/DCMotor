#ifndef DCMotor_h
#define DCMotor_h

#define DC_MOTOR_ONE_DIR 0
#define DC_MOTOR_ONE_DIR_PWM 1
#define DC_MOTOR_TWO_DIR 2
#define DC_MOTOR_TWO_DIR_PWM 3

#define DC_MOTOR_IDLE 0
#define DC_MOTOR_STARTING_A 1
#define DC_MOTOR_RUNNING_A 2
#define DC_MOTOR_STARTING_B 3
#define DC_MOTOR_RUNNING_B 4
#define DC_MOTOR_STOPPING 5

#include <inttypes.h>
#include <elapsedMillis.h>

struct DCMotorParam {
	int8_t mode = DC_MOTOR_ONE_DIR;
	unsigned long startStopDelay = 0; //[ms]
	unsigned long ramp = 0; //[ms -> 100%]
	unsigned long startRamp = 0; //[ms -> 100%]
	unsigned long stopRamp = 0; //[ms -> 100%]
	int8_t enaPin = -1;
	int8_t enbPin = -1;
	int8_t pwmPin = -1;
	int8_t diagPin = -1;
	int8_t csPin = -1;
};

class DCMotor {

public:

	//Constructors
	void begin();
	void begin(DCMotorParam parameters);

	//Commands
	void cmdA(); //Command to move in direction A, use previous or default speed and ramp time
	void cmdB(); //Command to move in direction B, use previous or default speed and ramp time
	void cmdA(uint8_t setPoint); //Command to move in direction A, use previous or default ramp time
	void cmdB(uint8_t setPoint); //Command to move in direction B, use previous or default ramp time
	void cmdA(uint8_t setPoint, unsigned long rampTime); //Command to move in direction A with specified ramp time
	void cmdB(uint8_t setPoint, unsigned long setRamp); //Command to move in direction B with specified ramp time
	void cmdStop(); //Command to stop with default ramp time
	void cmdStop(unsigned long setRamp); //Command to stop with specified ramp time
	
	//Settings
	void setStartRamp(unsigned long setRamp); //Set start ramp without issuing a start command
	void setStopRamp(unsigned long setRamp); //Set stop ramp without issuing a stop command
	void setSpeed(uint8_t setPoint);
	
	//Update
	void update();

	//Getters
	bool getInA();
	bool getInB();
	uint8_t getSpeed();
	uint8_t getState();
	float getRampConstant();
	
private:

	//State machine
	bool _newState = false;
	uint8_t _state = 0;
	uint8_t _nextState = 0;
	uint8_t _prevState = 0;
	elapsedMillis _stateTimer = 0;
	
	//Process values
	bool _inA = false;
	bool _inB = false;
	uint8_t _speed = 0;
	float _setSpeed_f = 0.0;
	float _startSpeed = 0.0; //Speed at start of acceleration / deceleration
	float _rampConstant = 0.0;
	float _jerkConstant = 0.0 ;
	float _speed_f = 0.0;
	bool _speedWithinMargin= false;
	
	//Settings
	int8_t _pwmPin = -1;
	int8_t _inaPin = -1;
	int8_t _inbPin = -1;
	int8_t _diagPin = -1;
	int8_t _csPin = -1;
	unsigned long _delay = 0;
	
	//Commands
	bool _cmdA = false;
	bool _cmdB = false;
	bool _cmdStop = false;
	
	//Setpoints
	unsigned long _startRampA = 0;
	unsigned long _startRampB = 0;
	unsigned long _stopRamp = 0;
	
	//Constants - states
	const uint8_t cIDLE = 0;
	const uint8_t cSTARTING_A = 1;
	const uint8_t cRUNNING_A = 2;
	const uint8_t cSTARTING_B = 3;
	const uint8_t cRUNNING_B = 4;
	const uint8_t cSTOPPING = 5;	
	
};
#endif