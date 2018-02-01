/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.commands.ChangeGears;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Pneumatics Subsystem
 */
public class Pneumatics extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	/**
	 * A method to turn a solenoid on or off
	 * @param on - true means on, false means off
	 */
	public static void setSolenoid1(boolean on) {
		RobotMap.SLN_rightSolenoid1.set(on);
	}
	
	/**
	 * A method to turn a solenoid on or off
	 * @param on - true means on, false means off
	 */
	public static void setSolenoid2(boolean on) {
		RobotMap.SLN_rightSolenoid2.set(on);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ChangeGears());
	}
}
