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
 * Gripper Subsystem
 */
public class Gripper extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	/**
	 * A method to pull or push the gripper motors based on speed (Negatives for
	 * pulling, positive for pushing)
	 * 
	 * @param speed
	 *            The speed of pull/push (-1 to 1)
	 */
	public static void pullPush(double speed) {
		RobotMap.TAL_gripMotorLeft.set(speed);
		RobotMap.TAL_gripMotorRight.set(speed * -1);
	}
	
	public static void stopGripper() {
		RobotMap.TAL_gripMotorLeft.set(0);
		RobotMap.TAL_gripMotorRight.set(0);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

}
