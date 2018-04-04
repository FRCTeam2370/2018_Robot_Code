package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousDefault extends CommandGroup {

	public AutonomousDefault() {
		// Drive forward into a wall or whatever
		

	
//		addSequential(new DriveBackwards(40), 2);
//		addSequential(new ElevatorToTop());
//		addSequential(new TurnRight(83), 3);
//		
//		addSequential(new DriveForwardsSlow(15), 3);
//		//addSequential(new PushGripper());
//		addSequential(new DriveBackwardsSlow(8), 2);
//		addSequential(new ElevatorToBottom());
		
		
		
		
	//working right side
	//addSequential(new DriveRight45(55), 3);
	//addSequential(new DriveLeft45(40), 3);
	//addSequential(new PushElevatorSol());
		
	//left side
	//the driveLeft and driveRight commands has 3 parameters (distance, angle, speed)
	addSequential(new DriveLeft(35, 45, .4), 3);
	addSequential(new DriveRight(45, 45, .4), 3);
	addSequential(new PushElevatorSol());
	
	}
}
