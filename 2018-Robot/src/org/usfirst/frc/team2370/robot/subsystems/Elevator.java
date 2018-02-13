/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;;

/**
 * Elevator Subsystem
 */
public class Elevator extends PIDSubsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public static final double BOTTOM = 0;

	public static final double p = 0.02;
	public static final double i = 0.0;
	public static final double d = 0.0;

	public Elevator() {
		super("Elevator", p, i, d);
		setAbsoluteTolerance(0.05);
		getPIDController().setContinuous(false);

		/*
		 * int timeout = 1000;
		 * RobotMap.elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.
		 * QuadEncoder, 0, timeout);
		 * RobotMap.elevatorMotor.getSensorCollection().setQuadraturePosition(0,
		 * timeout);
		 * 
		 * RobotMap.elevatorMotor.config_kP(0, p, timeout);
		 * RobotMap.elevatorMotor.config_kI(0, i, timeout);
		 * RobotMap.elevatorMotor.config_kD(0, d, timeout);
		 */
	}

	/**
	 * Set the PID setpoint to the given value
	 * 
	 * @param pos
	 *            The setpoint
	 */
	public static void setPos(double pos) {
		// RobotMap.elevatorMotor.set(ControlMode.Position, pos);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	@Override
	protected double returnPIDInput() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	protected void usePIDOutput(double output) {
		// TODO Auto-generated method stub

	}

}
