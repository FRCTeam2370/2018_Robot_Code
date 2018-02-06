/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Ramps Subsystem
 */
public class Ramps extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	// requires(good programming);

	public void winchWheels() {

	}

	/**
	 * This method changes the ramp position
	 * 
	 * @param rampDropped
	 *            true being down and false being raised
	 */
	public static void rampPosition(boolean rampDropped) {
		RobotMap.rampState = rampDropped;

	}

	/**
	 * A method to return the current state of the ramp, true being down and false
	 * being raised
	 * 
	 * @return
	 */
	public static boolean getRampState() {
		return RobotMap.rampState;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
