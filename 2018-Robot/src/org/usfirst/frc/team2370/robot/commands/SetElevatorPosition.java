package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetElevatorPosition extends Command {
	double position;
    public SetElevatorPosition(double position) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.position = position;
    	requires(Robot.kElevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(1.5);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Elevator.setPos(position);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
