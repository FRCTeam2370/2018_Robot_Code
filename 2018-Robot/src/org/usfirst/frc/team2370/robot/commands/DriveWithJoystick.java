/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.subsystems.DriveTrain;

/**
 * An example command. You can replace me with your own command.
 */
public class DriveWithJoystick extends Command {
	public DriveWithJoystick() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kDriveTrain);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		DriveTrain.arcadeDrive(0, 0);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (RobotMap.controller.getRawAxis(3) > RobotMap.deadbandPercent) {
			DriveTrain.arcadeDrive(RobotMap.controller.getRawAxis(3), RobotMap.controller.getRawAxis(0) );
		}
		else if (RobotMap.controller.getRawAxis(2) > RobotMap.deadbandPercent) {
			DriveTrain.arcadeDrive(RobotMap.controller.getRawAxis(2) *-1, RobotMap.controller.getRawAxis(0) );
		}
		else{
			DriveTrain.arcadeDrive(0, RobotMap.controller.getRawAxis(0));
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		// DriveTrain.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
