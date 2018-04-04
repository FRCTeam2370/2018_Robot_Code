package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousLeft extends CommandGroup {

	public AutonomousLeft() {
		switch (Robot.fieldMessage) {
		case "lll":
			if (RobotMap.preference.getVoltage() > RobotMap.preferenceAmount) {
				// Place the cube on the switch if you are on the right.
				addSequential(new DriveStraight(142), 3.25);
				addParallel(new PushElevatorSol(), 0.1);
				// addParallel(new SetElevatorPosition(-3000));
				addSequential(new TurnRight(84), 1.5);
				addSequential(new DriveStraight(18), 1.5);
				addSequential(new PushGripper());
				addSequential(new DriveBackwards(20), 3);
				// addSequential(new ElevatorToBottom());
				addSequential(new TurnLeft(84), 2.5);
				addSequential(new DriveStraight(5), 2);
			} else {
				addParallel(new PushElevatorSol());
				addSequential(new DriveStraight(242), 4);
				
				addParallel(new ElevatorToTop());
				addSequential(new TurnRight(45), 2.5);
				addSequential(new DriveForwardsSlow(15), 2.5);
				addSequential(new PushGripperButWayHarder(), 2);
				addSequential(new DriveBackwardsSlow(8), 2);
				addSequential(new ElevatorToBottom());
				addSequential(new TurnRight(75), 2.25);
				
				addSequential(new DriveStraight(28), 2); 
			}
			break;
		case "rrr":
			// Drive to the scale and place the cube (Around the world)
			addSequential(new DriveStraight(150), 4.3);
//			addSequential(new TurnRight(84), 1.5);
//			addSequential(new DriveStraight(156), 3.4);
//			addSequential(new PushElevatorSol(), .1);
//			addSequential(new TurnRight(65), 1);
//			// addSequential(new SetElevatorPosition(-3000));
//			addSequential(new DriveStraight(14), 1);
//			addSequential(new PushGripperButWayHarder());
			break;

		case "rlr":
			if (RobotMap.preference.getVoltage() > RobotMap.preferenceAmount) {
				// Drive to the scale and place the cube (Around the world)
				addSequential(new DriveStraight(150), 4.3);
//				addSequential(new TurnRight(84), 1.5);
//				addSequential(new DriveStraight(156), 3.4);
//				addSequential(new PushElevatorSol(), .1);
//				addSequential(new TurnRight(65), 1);
//				// addSequential(new SetElevatorPosition(-3000));
//				addSequential(new DriveStraight(14), 1);
//				addSequential(new PushGripperButWayHarder());
			} else {
				addParallel(new PushElevatorSol());
				addSequential(new DriveStraight(242), 4);
				
				addParallel(new ElevatorToTop());
				addSequential(new TurnRight(45), 2.5);
				addSequential(new DriveForwardsSlow(15), 2.5);
				addSequential(new PushGripperButWayHarder(), 2);
				addSequential(new DriveBackwardsSlow(8), 2);
				addSequential(new ElevatorToBottom());
				addSequential(new TurnRight(75), 2.25);
				
				addSequential(new DriveStraight(28), 2); 
			}
			break;

		case "lrl":
			
			/*addSequential(new DriveStraight(142), 3.25);
			addParallel(new PushElevatorSol(), .05);
			addSequential(new TurnRight(90), 1.5);
			addSequential(new DriveStraight(154), 3.7);
			addSequential(new TurnLeft(90), 1.5);*/
			
		
			// Place the cube on the switch if you are on the right.
			addSequential(new DriveStraight(142), 3.25);
			addParallel(new PushElevatorSol(), 0.1);
			// addParallel(new SetElevatorPosition(-3000));
			addSequential(new TurnRight(84), 1.5);
			addSequential(new DriveStraight(18), 1.5);
			addSequential(new PushGripper());
			addSequential(new DriveBackwards(20), 3);
			// addSequential(new ElevatorToBottom());
			addSequential(new TurnLeft(84), 2.5);
			addSequential(new DriveStraight(5), 2);
			break;
			
		default:
			addSequential(new DriveStraight(150), 5);
			break;
		}
	}
}
