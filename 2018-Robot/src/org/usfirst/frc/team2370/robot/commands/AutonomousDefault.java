package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousDefault extends CommandGroup {

	public AutonomousDefault() {
		/*
		addSequential(new DriveStraight(210), 4.3);
		addSequential(new TurnLeft(80), 1.5);
		addParallel(new PushElevatorSol(), .1);
		addSequential(new DriveStraight(165), 3.4);
		addSequential(new ElevatorToTop(), 1);
		addSequential(new TurnRight(80), 1.5);
		
		// addSequential(new SetElevatorPosition(-3000));
		addSequential(new DriveForwardsSlow(20), 4);
		addSequential(new PushGripperButWayHarder());
		addSequential(new DriveBackwardsSlow(14), 1.5);
		addSequential(new ElevatorToBottom(),1);
		*/
		
		
		addSequential(new DriveStraight(210), 3);
		addSequential(new TurnRight(80), 1.3);
		addParallel(new PushElevatorSol(), .5);
		addSequential(new DriveStraight(165), 3.6);
		addParallel(new ElevatorToTop(), .5);
		addSequential(new TurnLeft(80), 1.4);
		
		addSequential(new DriveForwardsSlow(28), 1.4);
		addSequential(new PushGripperButWayHarder(), .2);
		addSequential(new DriveBackwardsSlow(3), 1);
		addSequential(new ElevatorToBottom(), .5);
		
		

	//left side
	//the driveLeft and driveRight commands has 3 parameters (distance, angle, speed)
//addSequential(new PushElevatorSol(), .1);
//	addSequential(new DriveLeft(40, 45, .4), 1.3);
//	addSequential(new DriveRight(45, 45, .4), 2);
	
	//right side
	//addSequential(new PushElevatorSol(), .1);
	//addSequential(new DriveRight(50, 45, .4), 2);
	//addSequential(new DriveLeft(45, 45, .4), 2);
	
	
	}
}
