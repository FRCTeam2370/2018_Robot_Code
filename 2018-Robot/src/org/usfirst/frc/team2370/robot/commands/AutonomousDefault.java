package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousDefault extends CommandGroup {

	public AutonomousDefault() {
		// Drive forward into a wall or whatever
		

		addParallel(new PushElevatorSol());
		addSequential(new DriveStraight(247), 4);
		
		addParallel(new ElevatorToTop());
		addSequential(new TurnLeft(45), 1.5);
		addSequential(new DriveForwardsSlow(9), 2.25);
		//addSequential(new PushGripper(), 2);
		addSequential(new DriveBackwardsSlow(8), 2);
		addSequential(new ElevatorToBottom());
		addSequential(new TurnLeft(75), 2.25);
		
		addSequential(new DriveStraight(28), 2);
		
//		addSequential(new DriveBackwards(40), 2);
//		addSequential(new ElevatorToTop());
//		addSequential(new TurnRight(83), 3);
//		
//		addSequential(new DriveForwardsSlow(15), 3);
//		//addSequential(new PushGripper());
//		addSequential(new DriveBackwardsSlow(8), 2);
//		addSequential(new ElevatorToBottom());
		
		
		
		
		
	}
}
