/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2370.robot.subsystems.Elevator;

/**
 * An example command. You can replace me with your own command.
 */
public class CarrageToTop extends Command {
	public CarrageToTop() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.kElevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		//if (RobotMap.controller.getRawAxis(5) > RobotMap.deadbandPercent) {
		
		//Elevator.moveCarriage(RobotMap.controller.getRawAxis(5));
		
		if(RobotMap.TAL_carriageMotor.getSensorCollection().isRevLimitSwitchClosed()) {
		Elevator.stopCarriage();
		}else {
			Elevator.moveCarriage(1);
		}
		}
		
		//}
		/*else if (RobotMap.controller.getRawAxis(5) < (RobotMap.deadbandPercent) * -1) {
			Elevator.moveCarriage(RobotMap.controller.getRawAxis(5));
		}
		else{
			Elevator.stopCarriage();
		}*/
	

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		// DriveTrain.stopMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
