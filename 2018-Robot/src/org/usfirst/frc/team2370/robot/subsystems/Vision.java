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
 * Vision Subsystem
 */

public class Vision extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	/**
	 * Gets the value of a specific key sent back from the Limelight, keys found on
	 * the Limelight website
	 * 
	 * @param key
	 *            The String key used to name values
	 * @return Value contained in key from Limelight
	 */
	@SuppressWarnings("deprecation")
	public static double getLimeLightVal(String key) {
		return RobotMap.limeLightTable.getNumber(key, 0);
	}

	/**
	 * Sets up USB camera to a specific resolution 
	 * Possibly add FPS or similar settings here
	 */
	public static void usbCamSetup() {
		RobotMap.camera.setResolution(640, 480);
	}

	public void initDefaultCommand() {

	}
}
