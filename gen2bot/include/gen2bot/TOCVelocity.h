#pragma once

// This class is for declaring functions to run trencher and bucket motors

#include "ros/ros.h"
#include <iostream>
#include <std::string>

class TOCVelocity
{

public:
	TOCVelocity(ros::NodeHandle nh);

	void ConfigMotionMagic(TalonFX* talon1, int vel, int accel, int pos);
	void ConfigMotionMagic(TalonSRX* talon1, TalonSRX* talon2, int vel, int accel, int pos);

	void displayData(TalonFX* talon1, std::string name);
	void displayData(TalonSRX* talon1, TalonSRX* talon2, std::string name);

	bool ReverseLimitSwitchTriggered(TalonFX* talon1, std::string name);
	bool ReverseLimitSwitchTriggered(TalonSRX* talon1, TalonSRX* talon2, std::string name);

	bool isNear(int a, int b, int tolerance);

	bool TargetPositionReached(TalonFX* talon1, int pos, std::string name);
	bool TargetPositionReached(TalonSRX* talon1, TalonSRX* talon2, int pos, std::string name);

	bool CheckMode(int laPos, int buPos, int bsPos);

	void zeroStart(int& p_cmd, ros::NodeHandle  nh);
	void zero(int& p_cmd, ros::NodeHandle  nh);

	void driveMode(int& p_cmd, ros::NodeHandle  nh);
	void deposit(int& p_cmd, ros::NodeHandle  nh);
	void dig(int& p_cmd, ros::NodeHandle  nh);

	void checkSentinel(int& p_cmd);
	void isSafe(int& p_cmd);

	void stop();

	void config(ros::NodeHandle nh);

	void turnTrencher(int& p_cmd, ros::NodeHandle  nh);


	int sentinel;

	int mBuffer;

	// la = linear actuator
	double laDrivePosition;
	double laDepositPosition;
	double laDigPosition;

	// bs = ballscrew
	double bsDrivePosition;
	double bsDepositPosition;
	double bsDigPosition;

	// bu = bucket
	double buDrivePosition;
	double buDepositPosition;
	double buDigPosition;

	double trencherZeroPosition;
};
