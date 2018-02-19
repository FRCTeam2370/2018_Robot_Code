/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reser

ved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot;
 
import org.usfirst.frc.team2370.robot.commands.ChangeGears;
import org.usfirst.frc.team2370.robot.commands.DriveBackwards;
import org.usfirst.frc.team2370.robot.commands.DriveStraight;
import org.usfirst.frc.team2370.robot.commands.DropElevatorSol;
import org.usfirst.frc.team2370.robot.commands.ElevatorToBottom;
import org.usfirst.frc.team2370.robot.commands.ElevatorToTop;
import org.usfirst.frc.team2370.robot.commands.PullGripper;
import org.usfirst.frc.team2370.robot.commands.PushElevatorSol;
import org.usfirst.frc.team2370.robot.commands.PushGripper;
import org.usfirst.frc.team2370.robot.commands.StopGripper;
import org.usfirst.frc.team2370.robot.commands.TurnLeft;
import org.usfirst.frc.team2370.robot.commands.TurnRight;

//import edu.wpi.first.wpilibj.Joystick; 
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Operator interface
 */
public class OI {

	public OI() { 

		RobotMap.BTN_dropElevatorSol.whenPressed(new DropElevatorSol());
		RobotMap.BTN_pushElevatorSol.whenPressed(new PushElevatorSol());
		
		RobotMap.BTN_elevatorToBot.whenPressed(new ElevatorToBottom());
		RobotMap.BTN_elevatorToTop.whenPressed(new ElevatorToTop());
		
		RobotMap.BTN_gripperPull.whenReleased(new PullGripper());
		RobotMap.BTN_gripperPush.whenReleased(new PushGripper());
		
		RobotMap.BTN_gripperPull.whenInactive(new StopGripper());
		RobotMap.BTN_gripperPush.whenInactive(new StopGripper());
		
		//RobotMap.BTN_driveStraight.whenPressed(new DriveStraight(120));
		
		RobotMap.BTN_shift.whenPressed(new ChangeGears());//new DriveStraight(100));
	}
}
