/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;;

/**
 * Elevator Subsystem
 */
public class Elevator extends PIDSubsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public Elevator(double p, double i, double d) {
		super(p, i, d);
		
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	protected double returnPIDInput() {
		return 0;
	}

	@Override
	protected void usePIDOutput(double output) {
		
	}
}
