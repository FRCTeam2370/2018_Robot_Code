package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousLeft extends CommandGroup {

    public AutonomousLeft() {
        switch(RobotMap.fieldMessage) {
        case "lll": 
        	addSequential(new ElevatorToBottom());
        	addParallel(new DriveStraight(0));
        	addParallel(new PullGripper());
        	break;
        case "rrr":
        	addSequential(new ElevatorToBottom());
        	addParallel(new DriveStraight(0));
        	addParallel(new PullGripper());
        	break;
        case "rlr":
        	addSequential(new ElevatorToBottom());
        	addParallel(new DriveStraight(0));
        	addParallel(new PullGripper());
        	break;
        case "lrl":
        	addSequential(new ElevatorToBottom());
        	addParallel(new DriveStraight(0));
        	addParallel(new PullGripper());
        	break;
        }
    }
}
