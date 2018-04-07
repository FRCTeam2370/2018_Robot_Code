/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2370.robot.subsystems.Elevator;

/**
 * An example command. You can replace me with your own command.
 */
public class CarriageToTop extends Command {
	public CarriageToTop() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kElevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		Elevator.moveCarriage(1);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return RobotMap.TAL_carriageMotor.getSensorCollection().isRevLimitSwitchClosed();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Elevator.stopCarriage();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
