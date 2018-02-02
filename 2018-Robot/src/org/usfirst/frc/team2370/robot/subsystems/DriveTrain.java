/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.commands.DriveWithJoystick;

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
	public static void motorSetup() {
		RobotMap.TAL_leftSlave.follow(RobotMap.TAL_leftMaster);
		RobotMap.TAL_rightSlave.follow(RobotMap.TAL_rightMaster);
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

	}

	/**
	 * A method to turn the robot a specific number of degrees based on
	 * accelerometer
	 * 
	 * @param angle
	 *            The angle the robot will turn (Only positive angles)
	 */
	public static void turnLeft(double angle) {

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
	public static void driveStraight(double distance) {

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}
}
