/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
/*
 * Naming conventions
 * Button --> BTN
 * TalonSRX --> TAL
 * Solenoid --> SLN
 */
public class RobotMap {
	
	/**
	 * This will set the solenoids and compressor for the Pneumatics subsystem
	 */
	public static Solenoid SLN_rightSolenoid1 = new Solenoid(0);
	public static Solenoid SLN_rightSolenoid2 = new Solenoid(1);
	public static Compressor compressor = new Compressor();

	/**
	 * This will set the speed controllers, drive train object, and deadband for the DriveTrain subsystem
	 */
	public static WPI_TalonSRX TAL_rightMaster = new WPI_TalonSRX(16);
	public static WPI_TalonSRX TAL_leftMaster = new WPI_TalonSRX(17);
	public static WPI_TalonSRX TAL_rightSlave = new WPI_TalonSRX(18);
	public static WPI_TalonSRX TAL_leftSlave = new WPI_TalonSRX(19);
	public static DifferentialDrive driveTrain = new DifferentialDrive(TAL_leftMaster, TAL_rightMaster);
	public static double deadbandPercent = 0;
	
	/**
	 * This will set the speed controllers for the Gripper subsystem
	 */
	public static WPI_TalonSRX TAL_gripMotorLeft = new WPI_TalonSRX(0);
	public static WPI_TalonSRX TAL_gripMotorRight = new WPI_TalonSRX(0);
	
	/**
	 * This will set the speed controller for the Elevator subsystem
	 */
	public static WPI_TalonSRX TAL_elevatorMotor = new WPI_TalonSRX(12);
	
	/**
	 * This will set the Limelight network table up and the driver USB camera for the Vision subsystem
	 */
	public static NetworkTable limeLightTable = NetworkTable.getTable("limelight");
	public static UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

	/**
	 * This will set up the buttons and controller for the OI (Operator Interface)
	 */
	public static Joystick controller = new Joystick(0);
	public static Button BTN_elevToBot = new JoystickButton(controller, 1);
	public static Button BTN_elevToTop = new JoystickButton(controller, 2);
	public static Button BTN_shiftHigh = new JoystickButton(controller, 3);
	public static Button BTN_shiftLow = new JoystickButton(controller, 4);
	
}
