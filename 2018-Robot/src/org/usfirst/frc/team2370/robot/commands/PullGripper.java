/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.subsystems.Gripper;

/**
 * An example command. You can replace me with your own command.
 */
public class PullGripper extends Command {
	public PullGripper() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kGripper);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		setTimeout(1.5);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Gripper.pullPush(-.75);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Gripper.pullPush(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		//end();
	}
}
