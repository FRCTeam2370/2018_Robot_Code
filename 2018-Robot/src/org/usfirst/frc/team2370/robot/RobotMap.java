/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.mindsensors.CANLight;
import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
/*
 * Naming conventions Button --> BTN TalonSRX --> TAL Solenoid --> SLN
 */
@SuppressWarnings("deprecation")
public class RobotMap {

	/**
	 * This gets the game message to determine where our scale side is. 
	 */
	/**
	 * This will set the solenoids and compressor for the Pneumatics subsystem
	 */
	public static Solenoid SLN_shiftingSolenoid = new Solenoid(0);
	public static Compressor compressor = new Compressor();
	public static AnalogInput ALG_PreSensor = new AnalogInput(0);
	//public static AnalogInput ALA_BoxSensor = new AnalogInput(0);
	public static boolean driverStop = false;
	public static boolean doneDriving = false;
	public static boolean doneTurning = false;

	/**
	 * This will set the speed controllers, drive train object, and deadband for the
	 * DriveTrain subsystem
	 */
	public static WPI_TalonSRX TAL_rightMaster = new WPI_TalonSRX(13);
	public static WPI_TalonSRX TAL_leftMaster = new WPI_TalonSRX(11);
	public static WPI_TalonSRX TAL_rightSlave = new WPI_TalonSRX(12);
	public static WPI_TalonSRX TAL_leftSlave = new WPI_TalonSRX(10);
	public static DifferentialDrive driveTrain = new DifferentialDrive(TAL_leftMaster, TAL_rightMaster);
	public static double deadbandPercent = .05;
	public static double encoder2actual = 139.1519;//121.0576923;
	public static double encoder2TurnDegrees = 25.2888;
	public static double currentTurnDegrees;
	public static byte updateRate = 30;
	public static double oldAngle; 
	/**   
	 * This will set the speed controllers for the Gripper subsystem
	 */ 
	public static WPI_TalonSRX TAL_gripMotorLeft = new WPI_TalonSRX(14);
	public static WPI_TalonSRX TAL_gripMotorRight = new WPI_TalonSRX(6);
	
	public static double ThrottleDamper = 1;
	
	/**
	 * Motors for the Ramp to elevate after being dropped to the floor.
	 */
	
	//public static WPI_TalonSRX TAL_leftRampMotor = new WPI_TalonSRX(0);
	//public static WPI_TalonSRX TAL_rightRampMotor = new WPI_TalonSRX(0);

	/**
	 * This will set the speed controller for the Elevator subsystem
	 */

public static WPI_TalonSRX TAL_elevatorMotor = new WPI_TalonSRX(18);
	public static WPI_TalonSRX TAL_carriageMotor = new WPI_TalonSRX(3);
	public static Solenoid SLN_elevatorSolenoid = new Solenoid(1);
	public static DigitalInput DIG_elevatorBottom = new DigitalInput(0);
	public static DigitalInput DIG_elevatorTop = new DigitalInput(1);
	
	/*public static double pUp = 1.5;
	public static double pDown = 0.2;
	public static double i = 0.0;
	public static double d = 0.0;
	public static int timeout = 5;*/
	
	/**
	 * This will set the Limelight network table up and the driver USB  for
	 * the Vision subsystem
	 */
	public static NetworkTable limeLightTable = NetworkTable.getTable("limelight");
	
	/**
	 * This will set up the buttons and controller for the OI (Operator Interface)
	 */ 
	public static Joystick controller = new Joystick(0);
	public static Button BTN_elevatorToBot = new JoystickButton(controller, 1);
	public static Button BTN_elevatorToTop = new JoystickButton(controller, 2);
	public static Button BTN_shift = new JoystickButton(controller, 3);
	public static Button BTN_climb = new JoystickButton(controller, 4);
	public static Button BTN_gripperPull = new JoystickButton(controller, 5);
	public static Button BTN_gripperPush = new JoystickButton(controller, 6);
	public static Button BTN_dropElevatorSol = new JoystickButton(controller, 7);
	public static Button BTN_pushElevatorSol = new JoystickButton(controller, 8); 
	//public static Button BTN_driveStraight = new JoystickButton(controller, 1);
	/**
	 * This will set up Blinken if we end up using it for LEDs
	 * @param PWM channel
	 */
	 //public static PWM LED = new PWM(0);
	 /**
	  * This will set up the CANLight 
	  */
	 
	 //public static CANLight led = new CANLight(5);

	/**
	 * This will set up all NavX MXP objects
	 */
	public static AHRS ahrs;
	
	/**
	 * Digital inputs for autonomous choosing
	 */
	public static DigitalInput left = new DigitalInput(2);
	public static DigitalInput center = new DigitalInput(3);
	public static DigitalInput right = new DigitalInput(4);
	// Above 50% is switch preference, below 50% is scale preference
	public static double preferenceAmount = 2.5;
	public static AnalogInput preference = new AnalogInput(3);
	
	public static double originalAngle = 0;
	
	/**
	 * The most vital part of the robot, lighting effects
	 */
	// True only during autonomous
	//public static DigitalOutput autoPin = new DigitalOutput(6);
	// True if on blue team, false on red team
	public static DigitalOutput teamPin = new DigitalOutput(7);
	// True during last 30 seconds
	public static DigitalOutput timePin = new DigitalOutput(8);
	// True if box is inside of gripper
	public static DigitalOutput boxPin = new DigitalOutput(9);
	public static AnalogInput sonar = new AnalogInput(2);
}
