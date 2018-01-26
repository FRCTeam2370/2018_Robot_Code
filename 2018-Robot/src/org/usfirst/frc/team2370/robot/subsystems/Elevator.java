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
	public static final double BOTTOM = 0;

	public Elevator() {
		super("Elevator", 2.0, 0.0, 0.0);
		setAbsoluteTolerance(0.05);
		getPIDController().setContinuous(false);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	protected double returnPIDInput() {
    	//return .getAverageVoltage(); // returns the sensor value that is providing the feedback for the system
		return 0;
    }

	@Override
    protected void usePIDOutput(double output) {
    	//RobotMap.elevatorMotor.pidWrite(output); // this is where the computed output value fromthe PIDController is applied to the motor
    }
}
