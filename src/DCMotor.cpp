#include <Arduino.h>
#include <DCMotor.h>
#include <elapsedMillis.h>

//No parameters, dummy one direction
void DCMotor::begin() {
	_delay = 0;
	_state = cIDLE;
}

//Parameters supplied
void DCMotor::begin(DCMotorParam parameters) {
	_inaPin = parameters.enaPin;
	_inbPin = parameters.enbPin;
	_pwmPin = parameters.pwmPin;
	_diagPin = parameters.diagPin;
	_csPin = parameters.csPin;
	_pwmPin = parameters.pwmPin;
	_delay = parameters.startStopDelay;
	if (parameters.ramp > 0){
		_startRampA = parameters.ramp;
		_startRampB = parameters.ramp;
		_stopRamp = parameters.ramp;
	} else {
		_startRampA = parameters.startRamp;
		_startRampB = parameters.startRamp;
		_stopRamp = parameters.stopRamp;		
	}
	if (_inaPin > 0) pinMode(_inaPin, OUTPUT);
	if (_inbPin > 0) pinMode(_inbPin, OUTPUT);	
	if (_pwmPin > 0) pinMode(_pwmPin, OUTPUT);
	if (_diagPin > 0) pinMode(_diagPin, INPUT);
	if (_csPin > 0) pinMode(_csPin, INPUT);	
	_state = cIDLE;
}

//Command to move in direction A, use previous or default speed and ramp time
void DCMotor::cmdA(){
	_cmdA = true;	
}

//Command to move in direction A, use previous or default ramp time
void DCMotor::cmdA(uint8_t setPoint){
	_cmdA = true;	
	_setSpeed_f = (float)setPoint;
}

//Command to move in direction A with specified ramp time	
void DCMotor::cmdA(uint8_t setPoint, unsigned long rampTime) {
	_cmdA = true;
	_setSpeed_f = (float)setPoint;
	_startRampA = rampTime;
}

//Command to move in direction A, use previous or default speed and ramp time
void DCMotor::cmdB(){
	_cmdB = true;	
}

//Command to move in direction B, use previous or default ramp time
void DCMotor::cmdB(uint8_t setPoint){
	_cmdB = true;
	_setSpeed_f = (float)setPoint;	
}

//Command to move in direction B with specified ramp time
void DCMotor::cmdB(uint8_t setPoint, unsigned long rampTime) {
	_cmdB = true;
	_setSpeed_f = (float)setPoint;
	_startRampB = rampTime;
}

//Command to stop with default ramp time
void DCMotor::cmdStop(){
	_cmdStop = true;	
}

void DCMotor::setSpeed(uint8_t setPoint){
	_setSpeed_f = (float)setPoint;		
}

//Command to stop with specified ramp time
void DCMotor::cmdStop(unsigned long rampTime){
	_cmdStop = true;
	_stopRamp = rampTime;
}

//Set start ramp without issuing a stop command
void DCMotor::setStartRamp(unsigned long rampTime){
	
	_startRampA = rampTime;
	_startRampB = rampTime;
	_rampConstant = 255.0 / (float)_startRampA;
}

//Set stop ramp without issuing a stop command
void DCMotor::setStopRamp(unsigned long rampTime){
	_stopRamp = rampTime;
}

void DCMotor::update(){
	//Transitions
	if(_state == cIDLE && _cmdA && _stateTimer > _delay) _nextState = cSTARTING_A;
	if(_state == cIDLE && _cmdB && _stateTimer > _delay) _nextState = cSTARTING_B;
	if(_state == cSTARTING_A && _speedWithinMargin) _nextState = cRUNNING_A;
	if(_state == cSTARTING_B && _speedWithinMargin) _nextState = cRUNNING_B;
	if(_state == cSTARTING_A && (_cmdStop || _cmdB)) _nextState = cSTOPPING;
	if(_state == cSTARTING_B && (_cmdStop || _cmdA)) _nextState = cSTOPPING;
	if(_state == cRUNNING_A && (_cmdStop || _cmdB)) _nextState = cSTOPPING;
	if(_state == cRUNNING_B && (_cmdStop || _cmdA)) _nextState = cSTOPPING;	
	if(_state == cRUNNING_A && !_speedWithinMargin) _nextState = cSTARTING_A;
	if(_state == cRUNNING_B && !_speedWithinMargin) _nextState = cSTARTING_B;	
	if(_state == cSTOPPING && _speed <= 2) _nextState = cIDLE;
	
	//State machine
	_state = _nextState;
	_newState = _state != _prevState;
	if (_newState) _stateTimer = 0; 
	
	//Actions
	//IDLE
	if(_state == cIDLE && _newState){ //State entry
		_cmdStop = false;
		
		//Set enable
		_inA = false;
		_inB = false;
		
		//Set speeds
		_speed = 0;
		_speed_f = 0.0;
	}
	
	//STARTING A
	if(_state == cSTARTING_A && _newState){ //State entry
		
		//Set enable
		_inA = true;
		_inB = false;
		
		//Calculate ramp constant
		if (_startRampA > 0) {
			_rampConstant = 255.0 / (float)_startRampA;
		} else {
			_rampConstant = 0.0;	
		}
		
		//Store speed at start of acceleration
		_startSpeed = _speed_f;
		
		//Invert ramp if slowing down
		if(_startSpeed > _setSpeed_f) _rampConstant *= -1;
		
	}
	if(_state == cSTARTING_A){ //Continuous
	
		if(_startRampA > 0){
			_speed_f = _startSpeed + (_rampConstant * (float)_stateTimer);
			if(_setSpeed_f > _startSpeed){
				_speed_f = constrain(_speed_f, _startSpeed, _setSpeed_f); //Limit to set speed
			} else {
				_speed_f = constrain(_speed_f, _setSpeed_f, _startSpeed); //Limit to set speed				
			}
		} else {
			_speed_f = _setSpeed_f; //No ramp specified, jump directly to speed setpoint
		}
		//Convert speed and write to pin
		_speed = (int)_speed_f;	
	
	}

	//STARTING B
	if(_state == cSTARTING_B && _newState){ //State entry
		
		//Set enable
		_inA = false;
		_inB = true;
		
		//Calculate ramp constant
		if(_startRampB > 0) {
			_rampConstant = 255.0 / (float)_startRampB;
		} else {
			_rampConstant = 0.0;	
		}
		
		//Store speed at start of acceleration
		_startSpeed = _speed_f;
		
		//Invert ramp if slowing down
		if(_startSpeed > _setSpeed_f) _rampConstant *= -1;
		
	}
	if(_state == cSTARTING_B){ //Continuous
	
		if(_startRampB > 0){
			_speed_f = _startSpeed + (_rampConstant * (float)_stateTimer);
			if(_setSpeed_f > _startSpeed){
				_speed_f = constrain(_speed_f, _startSpeed, _setSpeed_f); //Limit to set speed
			} else {
				_speed_f = constrain(_speed_f, _setSpeed_f, _startSpeed); //Limit to set speed				
			}
		} else {
			_speed_f = _setSpeed_f; //No ramp specified, jump directly to speed setpoint
		}
		//Convert speed and write to pin
		_speed = (int)_speed_f;				
	}	
	
	//RUNNING A / RUNNING B
	if((_state == cRUNNING_A || _state == cRUNNING_B) && _newState) { // State entry

		//Write final speed setpoint
		_speed = (int)_setSpeed_f;			
	}

	//STOPPING
	if(_state == cSTOPPING && _newState){ //State entry
		if(_prevState == cSTARTING_A or _prevState == cRUNNING_A) _cmdA = false;
		if(_prevState == cSTARTING_B or _prevState == cRUNNING_B) _cmdB = false;		
		
		//Calculate ramp constant
		if (_stopRamp > 0) {
			_rampConstant = 255.0 / (float)_stopRamp;
		} else {
			_rampConstant = 0.0;	
		}
		
		//Store speed at start of deceleration
		_startSpeed = _speed_f;
	
	}
	if(_state == cSTOPPING){ //Continuous
	
		if(_stopRamp > 0){
			_speed_f = _startSpeed - (_rampConstant * (float)_stateTimer);
			_speed_f = constrain(_speed_f, 0.0, _startSpeed); //Limit between 0 and set speed
		} else {
			_speed_f = 0; //No ramp specified, jump directly to zero speed
		}
		//Convert speed and write to pin
		_speed = (int)_speed_f;
				
	}

	//Write outputs
	if(_inaPin > 0) digitalWrite(_inaPin, _inA);
	if(_inbPin > 0) digitalWrite(_inbPin, _inB);	
	if(_pwmPin > 0) analogWrite(_pwmPin, _speed);

	//Speed margin
	_speedWithinMargin = (_speed_f >= _setSpeed_f - 1.0 && _speed_f <= _setSpeed_f + 1.0);

	//Save previous state
	_prevState = _state;
	
}

uint8_t DCMotor::getSpeed(){
	return _speed;
}

uint8_t DCMotor::getState(){
	return _state;
}

float DCMotor::getRampConstant(){
	return _rampConstant;
}

bool DCMotor::getInA(){
	return _inA;
}

bool DCMotor::getInB(){
	return _inB;	
}
