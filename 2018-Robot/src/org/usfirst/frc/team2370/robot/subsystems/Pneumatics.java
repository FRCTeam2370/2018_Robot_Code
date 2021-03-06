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
	 * This will show if the gears are shifted to low or not, defaulting to low
	 */
	public static boolean isHigh = false;
	
	/**
	 * A method to shift gears
	 */
	public static void shiftGears() {
		if (isHigh == false) {
			RobotMap.SLN_shiftingSolenoid.set(true);
			isHigh = true;
		} else {
			RobotMap.SLN_shiftingSolenoid.set(false);
			isHigh = false;
		}
	}
	
	/**
	 * A method to extend elevator / set solenoid on or off
	 * 
	 * @param on, whether the solenoid is on or off
	 */
	public static void elevatorPush(boolean on) {
		RobotMap.SLN_elevatorSolenoid.set(on);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new ChangeGears());
	}
}
