package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorToBottom extends Command {

	public ElevatorToBottom() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.kElevator);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(0.1);
		//RobotMap.TAL_elevatorMotor.config_kP(0, RobotMap.pDown, RobotMap.timeout);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Elevator.setPos(0);
		//end();
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut();//false;
	}

	// Called once after isFinished returns true
	protected void end() {
	} 

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
