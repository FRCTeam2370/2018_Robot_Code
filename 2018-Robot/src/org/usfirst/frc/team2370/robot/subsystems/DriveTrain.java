/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Drive Train Subsystem
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	/**
	 * Method to setup the slave speed controllers to follower mode
	 */
	static double speed = 0.55;

	public static void motorSetup() {
		int timeout = 1000;
		double p = 1.0;

		RobotMap.TAL_leftSlave.follow(RobotMap.TAL_leftMaster);
		RobotMap.TAL_rightSlave.follow(RobotMap.TAL_rightMaster);

		RobotMap.TAL_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout);
		RobotMap.TAL_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout);
		RobotMap.TAL_leftMaster.getSensorCollection().setQuadraturePosition(0, timeout);
		RobotMap.TAL_rightMaster.getSensorCollection().setQuadraturePosition(0, timeout);
		RobotMap.TAL_leftMaster.config_kP(0, p, timeout);
		RobotMap.TAL_rightMaster.config_kP(0, p, timeout);

		/*
		 * RobotMap.TAL_rightMaster.setInverted(true);
		 * RobotMap.TAL_leftMaster.setInverted(true);
		 * RobotMap.TAL_rightSlave.setInverted(true);
		 * RobotMap.TAL_leftSlave.setInverted(true);
		 */

		try {
			RobotMap.ahrs = new AHRS(SerialPort.Port.kMXP, AHRS.SerialDataType.kRawData, RobotMap.updateRate);// kOnboard);
			RobotMap.ahrs.enableLogging(true);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		RobotMap.ahrs.reset();
	}

	/**
	 * Use arcade drive from the driveTrain using a speed and rotation parameter
	 * 
	 * @param speed
	 *            - The speed (forward and backward) of the speed controllers (-1.0
	 *            to 1.0)
	 * @param rotation
	 *            - The rotation (left and right) of the speed controllers (-1.0 to
	 *            1.0)
	 */
	public static void arcadeDrive(double speed, double rotation) {
		RobotMap.driveTrain.arcadeDrive(speed, rotation);
	}

	/**
	 * A method to stop all drive motors for use at the end of some commands
	 */
	public static void stopMotors() {
		RobotMap.TAL_rightMaster.set(0);
		RobotMap.TAL_leftMaster.set(0);
	}

	/**
	 * A method to turn the robot a specific number of degrees based on
	 * accelerometer
	 * 
	 * @param angle
	 *            The angle the robot will turn (Only positive angles)
	 */
	public static void turnRight(double angle) {
		RobotMap.currentTurnDegrees = ((RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition() + RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition()) /2)/RobotMap.encoder2TurnDegrees;
		if (RobotMap.currentTurnDegrees > angle*-1) {
			RobotMap.TAL_rightMaster.set(0.35);
			RobotMap.TAL_leftMaster.set(0.35);
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
	}

	/**
	 * A method to turn the robot a specific number of degrees based on
	 * accelerometer
	 * 
	 * @param angle
	 *            The angle the robot will turn (Only positive angles)
	 */
	public static void turnLeft(double angle) {
		RobotMap.currentTurnDegrees = ((RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition() + RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition()) /2)/RobotMap.encoder2TurnDegrees;
	
		if (RobotMap.currentTurnDegrees < angle) {
			RobotMap.TAL_rightMaster.set(-0.35);
			RobotMap.TAL_leftMaster.set(-0.35);
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
	}

	/**
	 * A method to turn the robot to a specific angle based on accelerometer
	 * 
	 * @param angle
	 *            The angle the robot will turn to in degrees
	 */
	public static void turnFieldOriented(double angle) {

	}

	/**
	 * A method to drive the robot forward a specific distance (In inches) using
	 * encoders
	 * 
	 * @param distance
	 *            The distance (In inches) to drive forward
	 */
	public static void driveForward(double distance) {

		// double fixedDistance = distance - 1;

		if ((RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() < distance
				* RobotMap.encoder2actual)
				&& RobotMap.TAL_leftMaster.getSensorCollection()
						.getQuadraturePosition() > (distance * RobotMap.encoder2actual) * -1) {
			RobotMap.TAL_rightMaster.set(-1 * speed);
			RobotMap.TAL_leftMaster.set(speed);
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
	}

	public static void driveBackwards(double distance) {
		if ((RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() > -distance
				* RobotMap.encoder2actual)
				&& RobotMap.TAL_leftMaster.getSensorCollection()
						.getQuadraturePosition() < (-distance * RobotMap.encoder2actual) * -1) {
			RobotMap.TAL_rightMaster.set(-1 * -speed);
			RobotMap.TAL_leftMaster.set(-speed);
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}
}
