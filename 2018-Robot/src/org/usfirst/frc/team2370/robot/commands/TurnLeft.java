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

public class TurnLeft extends Command {
	public double angle;
	
	public TurnLeft(double angle) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kDriveTrain);

		this.angle = angle;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//RobotMap.TAL_leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
		//RobotMap.TAL_rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
		RobotMap.oldAngle = RobotMap.ahrs.getAngle();
		//setTimeout(3);
		

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		DriveTrain.turnLeft(angle);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
