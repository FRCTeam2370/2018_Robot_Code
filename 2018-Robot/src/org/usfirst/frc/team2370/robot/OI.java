/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reser

ved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot;

import org.usfirst.frc.team2370.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team2370.robot.commands.ExampleDriveCommand;
import org.usfirst.frc.team2370.robot.commands.MoveElevatorTest;
import org.usfirst.frc.team2370.robot.commands.PullGripper;
import org.usfirst.frc.team2370.robot.commands.PushGripper;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

import org.usfirst.frc.team2370.robot.commands.ElevatorToBottom;
import org.usfirst.frc.team2370.robot.commands.ElevatorToTop;


//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Operator interface
 */
public class OI {

	public static void init() {
		
		
		RobotMap.BTN_elevToBot.whenPressed(new ElevatorToBottom());
		RobotMap.BTN_elevToTop.whenPressed(new ElevatorToTop());

		// button.whenPressed(new ExampleDriveCommand());

		// button.whileHeld(new ExampleDriveCommand());

		// button.whenReleased(new ExampleDriveCommand());
	}
}
