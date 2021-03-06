package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveBackwards extends Command {
	double distance;
    public DriveBackwards(double distance) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	this.distance = distance;
    	requires(Robot.kDriveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(5);
    	RobotMap.TAL_rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
		RobotMap.TAL_leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	DriveTrain.driveBackwards(distance);
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
