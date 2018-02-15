/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reser

ved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot;
 
import org.usfirst.frc.team2370.robot.commands.CarriageToBottom;
import org.usfirst.frc.team2370.robot.commands.CarriageToTop;
import org.usfirst.frc.team2370.robot.commands.ChangeGears;
import org.usfirst.frc.team2370.robot.commands.MoveElevator;
import org.usfirst.frc.team2370.robot.commands.PullGripper;
import org.usfirst.frc.team2370.robot.commands.PushGripper;

//import edu.wpi.first.wpilibj.Joystick; 
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Operator interface
 */
public class OI {

	public OI() { 

		RobotMap.BTN_carriageToBot.whenPressed(new CarriageToBottom());
		RobotMap.BTN_carriageToTop.whenPressed(new CarriageToTop());

		RobotMap.BTN_elevatorMove.whenPressed(new MoveElevator());
		
		RobotMap.BTN_gripperPull.whenPressed(new PullGripper());
		RobotMap.BTN_gripperPush.whenPressed(new PushGripper());
		
		RobotMap.BTN_shift.whenPressed(new ChangeGears());
	}
}
