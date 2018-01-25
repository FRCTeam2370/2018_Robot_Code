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
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;

	public static WPI_TalonSRX rightMaster = new WPI_TalonSRX(16);
	public static WPI_TalonSRX leftMaster = new WPI_TalonSRX(17);
	public static WPI_TalonSRX rightSlave = new WPI_TalonSRX(18);
	public static WPI_TalonSRX leftSlave = new WPI_TalonSRX(19);
	public static DifferentialDrive driveTrain = new DifferentialDrive(leftMaster, rightMaster);
	// public static RobotDrive driveTrain = new RobotDrive(leftMaster,
	// rightMaster);

	/*
	 * public static WPI_TalonSRX gripMotor = new WPI_TalonSRX(0);
	 * 
	 * public static WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(0);
	 * 
	 * //public static DigitalInput testDigitalInput = new DigitalInput(0); //public
	 * static DigitalOutput testDigitalOutput = new DigitalOutput(0);
	 * 
	 * public static Compressor compressor = new Compressor(0); public static
	 * Solenoid gearSolenoid1 = new Solenoid(0); public static Solenoid
	 * gearSolenoid2 = new Solenoid(1);
	 */
	public static NetworkTable limeLightTable = NetworkTable.getTable("limelight");
	public static UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

	public static Joystick stick = new Joystick(0);
	public static Button lbutton = new JoystickButton(RobotMap.stick, 5);
	public static Button rbutton = new JoystickButton(RobotMap.stick, 6);
}
