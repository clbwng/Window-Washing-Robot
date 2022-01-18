//WINDOW WASHING PROJECT 2021
//DESIGNED FOR LEGO EV3 PARTS

#define GYRO_SENSOR_PORT S3
#define ULTRASONIC_SENSOR_PORT S2
#define COLOR_SENSOR_PORT S1

#define VERTICAL_MOTOR motorD
#define HORIZONTAL_MOTOR_ONE motorA
#define HORIZONTAL_MOTOR_TWO motorB
#define SPINNER_MOTOR motorC

#define HORZ_RADIUS 2.5
#define VERT_RADIUS 1
#define TOLERANCE_OFFSET 1000

const int eraseSpeed = 100;
const int moveSpeed = 50;
const int sideMoveSpeed = 25;

const int eraserWidth = 5;

const float measurementMultiplier = 1.5;

int boardWidth = 0;
int boardHeight = 0;
const int cleanDelay = 10;//delay between each cleaning in seconds

const int rotateLimit = 45;
//max allowed rotation of robot before gyro indicates windy conditions
const float minUltraDistance = 15;
//min allowed distance from ultrasonic sensor

void configSensors()
{
	nMotorEncoder[HORIZONTAL_MOTOR_ONE] = nMotorEncoder[HORIZONTAL_MOTOR_TWO] = 0;
	nMotorEncoder[VERTICAL_MOTOR] = 0;
	nMotorEncoder[SPINNER_MOTOR] = 0;

	SensorType[GYRO_SENSOR_PORT] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[GYRO_SENSOR_PORT] = modeEV3Gyro_Calibration;
	wait1Msec(50);
	SensorMode[GYRO_SENSOR_PORT] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	resetGyro(GYRO_SENSOR_PORT);

	SensorType[ULTRASONIC_SENSOR_PORT] = sensorEV3_Ultrasonic;
	wait1Msec(50);

	SensorType[COLOR_SENSOR_PORT] = sensorEV3_Color;
	wait1Msec(50);
	SensorMode[COLOR_SENSOR_PORT] = modeEV3Color_Color;
	wait1Msec(50);
}


bool ifWindOrObstructions()
{
	if (getGyroDegrees(GYRO_SENSOR_PORT) >= rotateLimit || getGyroDegrees(GYRO_SENSOR_PORT) <= -rotateLimit
			|| SensorValue[ULTRASONIC_SENSOR_PORT] < minUltraDistance)
		return true;
	else
		return false;
}

//returns false if successful, true if interrupted
bool moveHorz(int dist_cm) {
    int enc_limit = (dist_cm*360)/(2*PI*HORZ_RADIUS);
    int current = nMotorEncoder[HORIZONTAL_MOTOR_TWO];
    bool interrupt = False;

    //if dist_cm greater than 0, robot moves right and vice versa

    if (dist_cm > 0) {
        motor[HORIZONTAL_MOTOR_ONE] = -moveSpeed;
        motor[HORIZONTAL_MOTOR_TWO] = moveSpeed;
        while(nMotorEncoder[HORIZONTAL_MOTOR_TWO] < current + enc_limit)
        {
            if(ifWindOrObstructions())
            {
            	motor[HORIZONTAL_MOTOR_ONE] = 0;
        			motor[HORIZONTAL_MOTOR_TWO] = 0;
            	return true;
          	}
        }
        motor[HORIZONTAL_MOTOR_ONE] = 0;
        motor[HORIZONTAL_MOTOR_TWO] = 0;
    }
    else if (dist_cm < 0) {
        motor[HORIZONTAL_MOTOR_ONE] = moveSpeed;
        motor[HORIZONTAL_MOTOR_TWO] = -moveSpeed;
        while(nMotorEncoder[HORIZONTAL_MOTOR_TWO] > current - enc_limit)
        {
            if(ifWindOrObstructions())
            {
            	motor[HORIZONTAL_MOTOR_ONE] = 0;
        			motor[HORIZONTAL_MOTOR_TWO] = 0;
            	return true;
          	}
        }
        motor[HORIZONTAL_MOTOR_ONE] = 0;
        motor[HORIZONTAL_MOTOR_TWO] = 0;
    }

    return false;

}

bool moveVert(int dist_cm) {
    int enc_limit = -(dist_cm*360)/(2*PI*VERT_RADIUS);
    int current = nMotorEncoder[VERTICAL_MOTOR];
    bool interrupt = False;
    resetMotorEncoder(VERTICAL_MOTOR);
    //if dist_cm greater than 0, robot moves down and vice versa
    if (dist_cm > 0) {
        motor[VERTICAL_MOTOR] = -moveSpeed;
        while(nMotorEncoder[VERTICAL_MOTOR] > enc_limit)
        {
        	if(ifWindOrObstructions())
            {
            	motor[VERTICAL_MOTOR] = 0;
            	return true;
          	}
        }
        motor[VERTICAL_MOTOR] = 0;
    }
    else if (dist_cm < 0) {
        motor[VERTICAL_MOTOR] = moveSpeed;
        while(nMotorEncoder[VERTICAL_MOTOR] < current - enc_limit)
        {
            if(ifWindOrObstructions())
            {
            	motor[VERTICAL_MOTOR] = 0;
            	return true;
          	}
        }
        motor[VERTICAL_MOTOR] = 0;
    }

    return false;

}


bool isBlack()
{
	if (SensorValue[COLOR_SENSOR_PORT] == (int)colorBlack)
		return true;
	else
		return false;
}

void moveToTop()
{
	while(nMotorEncoder(VERTICAL_MOTOR) < -TOLERANCE_OFFSET)
	{
		motor[VERTICAL_MOTOR] = moveSpeed;
	}
	while(!isBlack())
	{
		motor[VERTICAL_MOTOR] = moveSpeed;
	}
	motor[VERTICAL_MOTOR] = 0;
	resetMotorEncoder(VERTICAL_MOTOR);
}

void moveToTopLeft()
{
	moveToTop();
	while(isBlack())
	{
		motor[VERTICAL_MOTOR] = -moveSpeed;
	}
	wait1Msec(100);
	motor[VERTICAL_MOTOR] = 0;
	while(!isBlack())
	{
		motor[HORIZONTAL_MOTOR_ONE] = sideMoveSpeed;
		motor[HORIZONTAL_MOTOR_TWO] = -sideMoveSpeed;
	}
	motor[HORIZONTAL_MOTOR_ONE] = 0;
	motor[HORIZONTAL_MOTOR_TWO] = 0;
	resetMotorEncoder(HORIZONTAL_MOTOR_TWO);
}

void eraserMotor(bool enabled)
{
	motor[SPINNER_MOTOR] = eraseSpeed*enabled;
}

void manualMove()
{
	nMotorEncoder[HORIZONTAL_MOTOR_ONE] = nMotorEncoder[HORIZONTAL_MOTOR_TWO] = 0;
	nMotorEncoder[VERTICAL_MOTOR] = 0;
	displayString(5, "Setup mode, use arrow keys to move robot to bottom right corner");
	displayString(6, "Then press enter.  This sets the board dimensions");

	while(!getButtonPress(buttonEnter))
	{
		while(getButtonPress(buttonUp))
		{
			motor[VERTICAL_MOTOR] = moveSpeed;
		}
		while(getButtonPress(buttonDown))
		{
			motor[VERTICAL_MOTOR] = -moveSpeed;
		}
		while(getButtonPress(buttonLeft))
		{
			motor[HORIZONTAL_MOTOR_ONE] = moveSpeed;
			motor[HORIZONTAL_MOTOR_TWO] = -moveSpeed;
		}
		while(getButtonPress(buttonRight))
		{
			motor[HORIZONTAL_MOTOR_ONE] = -moveSpeed;
			motor[HORIZONTAL_MOTOR_TWO] = moveSpeed;
		}
		motor[HORIZONTAL_MOTOR_ONE] = 0;
		motor[HORIZONTAL_MOTOR_TWO] = 0;
		motor[VERTICAL_MOTOR] = 0;
	}
	int tempWidth = nMotorEncoder[HORIZONTAL_MOTOR_TWO];
	int tempHeight = -(nMotorEncoder[VERTICAL_MOTOR]);
	boardWidth = (tempWidth*2*Pi*HORZ_RADIUS)/360;
	boardHeight = (tempHeight*Pi*VERT_RADIUS)/360;

}

void displayDelay(int secs)
{
	for(int i = secs; i >= 0; i--)
	{
		displayString(3, "starting in %d seconds", i);
		wait1Msec(1000);
	}
}

task main()
{
	configSensors();
	moveToTopLeft();
	//manualMove() sets the board with and height manually
	manualMove();
	displayString(7, "w: %d, h: %d", boardWidth, boardHeight);
	displayDelay(3);
	moveToTopLeft();
	for(int j = 0; j < 2; j++)
	{
		ClearTimer(T1);
		int startTime = time1[T1]/1000;
		while(time1[T1]/1000 - startTime < cleanDelay && !getButtonPress(buttonEnter)){}
		moveHorz(5);
		for(int i = 0; i < boardWidth/eraserWidth; i++)
		{
			eraserMotor(true);
			if(moveVert(boardHeight*measurementMultiplier))
			{
				eraserMotor(false);
				break;
			}

			eraserMotor(false);

			moveToTop();
			if(i != boardWidth/eraserWidth-1)
			{
				moveHorz(eraserWidth);
			}
		}
		moveToTopLeft();
	}
}
