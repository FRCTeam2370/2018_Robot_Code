package org.usfirst.frc.team2370.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class GROUP_StartCenter extends CommandGroup{
	
	public GROUP_StartCenter() {
		
	//Need at least 140 inch equivalent to reach switch distance(Currently 0 as filler)
	addParallel(new DriveStraight(0));
	addParallel(new ElevatorToBottom());
	
	addSequential(new Turn());
	
	addSequential(new PullGripper());
	
	addSequential(new ElevatorToTop());
	
	
	}
}
