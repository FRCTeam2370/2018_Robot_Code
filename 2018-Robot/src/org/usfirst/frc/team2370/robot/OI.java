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

		// button.whenPressed(new ExampleDriveCommand());

		// button.whileHeld(new ExampleDriveCommand());

		// button.whenReleased(new ExampleDriveCommand());
	}
}
