package org.usfirst.frc.team2370.robot.commands;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonomousCenter extends CommandGroup {

	public AutonomousCenter() {
		/*switch (Robot.fieldMessage) {
		case "lll":
			addSequential(new DriveLeft(), 2);
			addParallel(new PushElevatorSol());
			addSequential(new DriveRight(), 1.6);
			addSequential(new PushGripper());
			break;
		case "rrr":
			addSequential(new DriveRight(), 1.9);
			addParallel(new PushElevatorSol());
			addSequential(new DriveLeft(), 1.6);
			addSequential(new PushGripper());
			break;
		case "rlr":
			addSequential(new DriveRight(), 1.9);
			addParallel(new PushElevatorSol());
			addSequential(new DriveLeft(), 1.6);
			addSequential(new PushGripper());
			break;
		case "lrl":
			addSequential(new DriveLeft(), 2);
			addParallel(new PushElevatorSol());
			addSequential(new DriveRight(), 1.6);
			addSequential(new PushGripper());
			break;
		}*/
		switch (Robot.fieldMessage) {
		case "lll":
			addSequential(new DriveLeft(0, 0, 0), 1.5);
			addParallel(new PushElevatorSol());
			addSequential(new DriveRight(0, 0, 0), 1.3);
			addSequential(new PushGripper());
			break;
		case "rrr":
			addSequential(new DriveRight(0, 0, 0), 1.4);
			
			break;
		case "rlr":
			addSequential(new DriveRight(0, 0, 0), 1.4);
			addParallel(new PushElevatorSol());
			addSequential(new DriveLeft(0, 0, 0), 1.2);
			addSequential(new PushGripper());
			break;
		case "lrl":
			addSequential(new DriveLeft(0, 0, 0), 1.5);
			addParallel(new PushElevatorSol());
			addSequential(new DriveRight(0, 0, 0), 1.3);
			addSequential(new PushGripper());
			break;
		}
	}
}
