/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reser

ved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package src.org.usfirst.frc.team2370.robot; 

import src.org.usfirst.frc.team2370.robot.commands.ElevatorToBottom;
import src.org.usfirst.frc.team2370.robot.commands.ElevatorToTop;

//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Operator interface
 */
public class OI {

	public static void init() {
<<<<<<< HEAD
		Button elevatorToBottom = new JoystickButton(RobotMap.stick, 1);
		Button elevatorToTop = new JoystickButton(RobotMap.stick, 2);
		Button shiftHighButton = new JoystickButton(RobotMap.stick, 3);
		Button shiftLowButton = new JoystickButton(RobotMap.stick, 4);
		Button lbutton = new JoystickButton(RobotMap.stick, 5);
		Button rbutton = new JoystickButton(RobotMap.stick, 6);
		 
		lbutton.whileHeld(new PullGripper());
		rbutton.whileHeld(new PushGripper());
		lbutton.whileHeld(new MoveElevatorTest());
		
		//You can switch these but uuuuuuuuh.
		elevatorToBottom.whileHeld(new ElevatorToBottom());
		elevatorToTop.whileHeld(new ElevatorToTop());
=======
>>>>>>> branch 'master' of https://github.com/FRCTeam2370/2018_Robot_Code.git

		RobotMap.BTN_elevToBot.whenPressed(new ElevatorToBottom());
		RobotMap.BTN_elevToTop.whenPressed(new ElevatorToTop());

		// button.whenPressed(new ExampleDriveCommand());

		// button.whileHeld(new ExampleDriveCommand());

		// button.whenReleased(new ExampleDriveCommand());
	}
}
