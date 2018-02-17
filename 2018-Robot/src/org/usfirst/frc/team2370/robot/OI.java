/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reser

ved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot;
 
import org.usfirst.frc.team2370.robot.commands.ChangeGears;
import org.usfirst.frc.team2370.robot.commands.ElevatorToBottom;
import org.usfirst.frc.team2370.robot.commands.ElevatorToTop;
import org.usfirst.frc.team2370.robot.commands.PullGripper;
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

		RobotMap.BTN_elevatorToBot.whenPressed(new ElevatorToBottom());//(new TurnRight(90));
		RobotMap.BTN_elevatorToTop.whenPressed(new ElevatorToTop());//new TurnLeft(90));
		
		RobotMap.BTN_gripperPull.whenPressed(new PullGripper());
		RobotMap.BTN_gripperPush.whenPressed(new PushGripper());
		
		//RobotMap.BTN_gripperPull.whenInactive(new StopGripper());
		//RobotMap.BTN_gripperPush.whenInactive(new StopGripper());
		
		RobotMap.BTN_shift.whenPressed(new ChangeGears());
	}
}
