package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutonomousRight extends CommandGroup {

	public AutonomousRight() {

		switch (Robot.fieldMessage) {

		case "lll":
			if (RobotMap.preference.getVoltage() < RobotMap.preferenceAmount) {

				addSequential(new DriveStraight(211));
				addSequential(new TurnLeft(80));
				addSequential(new PushElevatorSol(), .1);
				addSequential(new DriveStraight(175));
				addParallel(new CarriageToTop());
				addParallel(new ElevatorToTop(), .5);
				addSequential(new TurnRight(75));
				addSequential(new Wait(.5));
				addSequential(new DriveForwardsSlow(28), 1.2);
				addSequential(new PushGripperButWayHarder());
				addSequential(new DriveBackwardsSlow(3), .7);
				addSequential(new ElevatorToBottom(), .5);
				addSequential(new TurnRight(175));
				addSequential(new CarriageToBottom());

			} else {
				// Drive to the scale and place the cube (Around the world)
				addSequential(new DriveStraight(150));
				addSequential(new TurnLeft(84));
				addSequential(new DriveStraight(156));
				addSequential(new PushElevatorSol(), .1);
				addSequential(new TurnLeft(65));
				addSequential(new DriveStraight(14));
				addSequential(new PushGripperButWayHarder());
			}
			break;
			
			
		case "rrr":
			
			if (RobotMap.preference.getVoltage() < RobotMap.preferenceAmount) {
				addParallel(new PushElevatorSol());
				addSequential(new DriveStraight(242));
				
				addParallel(new ElevatorToTop());
				addSequential(new TurnLeft(45));
				addSequential(new DriveForwardsSlow(15), 2.5);
				addSequential(new PushGripperButWayHarder(), 2);
				addSequential(new DriveBackwardsSlow(8), 2);
				addSequential(new ElevatorToBottom());
				addSequential(new TurnLeft(75));
				addSequential(new DriveStraight(28));
			} else {
				// start
				addSequential(new DriveStraight(142), 3.25);
				addParallel(new PushElevatorSol(), 0.1);
				//addParallel(new SetElevatorPosition(-3000));
				addSequential(new TurnLeft(84), 1.5);
				addSequential(new DriveStraight(18), 1.5);
				addSequential(new PushGripper());
				addSequential(new DriveBackwards(20), 3);
				addSequential(new ElevatorToBottom());
				addSequential(new TurnRight(84), 2.5);
				addSequential(new DriveStraight(5), 2);
			}
			break;
		// stop
		case "lrl":
			if (RobotMap.preference.getVoltage() < RobotMap.preferenceAmount) {
				addParallel(new PushElevatorSol());
				addSequential(new DriveStraight(242), 4);
				
				addParallel(new ElevatorToTop());
				addSequential(new TurnLeft(45), 2.5);
				addSequential(new DriveForwardsSlow(15), 2.5);
				addSequential(new PushGripperButWayHarder(), 2);
				addSequential(new DriveBackwardsSlow(8), 2);
				addSequential(new ElevatorToBottom());
				addSequential(new TurnLeft(75), 2.25);
				
				addSequential(new DriveStraight(28), 2);
			} else {
				// Drive to the scale and place the cube (Around the world)
				addSequential(new DriveStraight(150));
				addSequential(new TurnLeft(84));
				addSequential(new DriveStraight(156));
				addSequential(new PushElevatorSol(), .1);
				addSequential(new TurnLeft(65));
				addSequential(new DriveStraight(14));
				addSequential(new PushGripperButWayHarder());
			}
			break;
		case "rlr":
			// start
			if (RobotMap.preference.getVoltage() < RobotMap.preferenceAmount) {


				addSequential(new DriveStraight(142));
				addParallel(new PushElevatorSol(), 0.1);
				//addParallel(new SetElevatorPosition(-3000));
				addSequential(new TurnLeft(84));
				addSequential(new DriveStraight(18));
				addSequential(new PushGripper());
				addSequential(new DriveBackwards(20), 3);
				addSequential(new ElevatorToBottom());
				addSequential(new TurnRight(84));
				addSequential(new DriveStraight(5));

			} else {
				addSequential(new DriveStraight(211));
				addSequential(new TurnLeft(80));
				addSequential(new PushElevatorSol(),.1);
				addSequential(new DriveStraight(175));
				addParallel(new CarriageToTop());
				addParallel(new ElevatorToTop(), .5);
				
				addSequential(new TurnRight(75));
				addSequential(new Wait(.5));
			
				addSequential(new DriveForwardsSlow(28), 1.2);
				addSequential(new PushGripperButWayHarder());
				addSequential(new DriveBackwardsSlow(3), .7);
				
				addSequential(new ElevatorToBottom(), .5);
				addSequential(new TurnLeft(175));
				addSequential(new CarriageToBottom());	
			}
			
			
			break;
		// stop
		default:
			addSequential(new DriveStraight(150), 15);
			break;
		}

	}

}
