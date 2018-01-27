/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.OI;
import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drive Train Subsystem
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public static void motorSetup() {
		RobotMap.leftSlave.follow(RobotMap.leftMaster);
		RobotMap.rightSlave.follow(RobotMap.rightMaster);
	}

	public static void arcadeDrive(double speed, double rotation) {
		RobotMap.driveTrain.arcadeDrive(speed, rotation);

	}

	public static void stopMotors() {
		RobotMap.rightMaster.set(0);
		RobotMap.leftMaster.set(0);
	}

	public static void turn(double angle) {

	}
	
	public static void driveStraight(double distance) {

	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}
}
