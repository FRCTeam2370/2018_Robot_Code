/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.commands.CarriageWithJoystick;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;;

/**
 * Elevator Subsystem
 */
public class Elevator extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public static final double BOTTOM = 0;

	public static final double p = 2.5;
	public static final double i = 0.0;
	public static final double d = 0.0;

	public Elevator() {
		
	}

	public static void elevatorSetup() {
		int timeout = 1000;
		RobotMap.TAL_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout);
		RobotMap.TAL_elevatorMotor.getSensorCollection().setQuadraturePosition(0, timeout);

		RobotMap.TAL_elevatorMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1, 0x00, 0x00, 0x00);
		RobotMap.TAL_elevatorMotor.configNominalOutputForward(0, timeout);
		RobotMap.TAL_elevatorMotor.configNominalOutputReverse(0, timeout);
		RobotMap.TAL_elevatorMotor.configPeakOutputForward(0.6, timeout);
		RobotMap.TAL_elevatorMotor.configPeakOutputReverse(-1.0, timeout);
		
		//RobotMap.TAL_elevatorMotor.
		
		RobotMap.TAL_elevatorMotor.config_kP(0, p, timeout);
		RobotMap.TAL_elevatorMotor.config_kI(0, i, timeout);
		RobotMap.TAL_elevatorMotor.config_kD(0, d, timeout);

		RobotMap.TAL_elevatorMotor.setInverted(true);
		
		//RobotMap.TAL_carriageMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
}
	
	/**
	 * Set the PID setpoint to the given value
	 * 
	 * @param pos
	 *            The setpoint
	 */
	public static void setPos(double pos) {
		RobotMap.TAL_elevatorMotor.set(ControlMode.Position, pos);
	}
	
public static double resetPos = 0;
	
	public static void elevatorReset() {
		if (RobotMap.DIG_elevatorBottom.get() == false) {
			//RobotMap.TAL_elevatorMotor.set(-0.1);
			RobotMap.TAL_elevatorMotor.set(ControlMode.Position, resetPos);
			resetPos += 30;
		} else {
			RobotMap.TAL_elevatorMotor.set(ControlMode.Position, 0);
			RobotMap.TAL_elevatorMotor.getSensorCollection().setQuadraturePosition(0, 20);
		}
	}
	
	public static void moveCarriage(double speed) {
//		if (RobotMap.TAL_carriageMotor.getSensorCollection().isFwdLimitSwitchClosed() == false && speed > 0) {
//			RobotMap.TAL_carriageMotor.set(speed);
//		}
//		else if (RobotMap.TAL_carriageMotor.getSensorCollection().isFwdLimitSwitchClosed() == true) {
//			RobotMap.TAL_carriageMotor.set(speed);
//		}
//		else {
//			RobotMap.TAL_carriageMotor.set(0);
//		}
		
		RobotMap.TAL_carriageMotor.set(speed);
		
	}
	
	public static void stopCarriage() {
		RobotMap.TAL_carriageMotor.set(0);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new CarriageWithJoystick());
	}

	
}
