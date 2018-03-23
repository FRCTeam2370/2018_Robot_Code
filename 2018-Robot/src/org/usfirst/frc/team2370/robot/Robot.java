/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//

//Heres a test message my mans

//

//

package org.usfirst.frc.team2370.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2370.robot.commands.AutonomousLeft;
import org.usfirst.frc.team2370.robot.commands.AutonomousRight;
//import org.usfirst.frc.team2370.robot.commands.testAuto;

import java.util.concurrent.TimeUnit;

import org.usfirst.frc.team2370.robot.commands.AutonomousCenter;
import org.usfirst.frc.team2370.robot.commands.AutonomousDefault;
import org.usfirst.frc.team2370.robot.commands.AutonomousLeft;
import org.usfirst.frc.team2370.robot.commands.AutonomousRight;

import org.usfirst.frc.team2370.robot.subsystems.Dashboard;
import org.usfirst.frc.team2370.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2370.robot.subsystems.Elevator;
import org.usfirst.frc.team2370.robot.subsystems.Gripper;
import org.usfirst.frc.team2370.robot.subsystems.LEDs;
import org.usfirst.frc.team2370.robot.subsystems.Pneumatics;
import org.usfirst.frc.team2370.robot.subsystems.Vision;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends TimedRobot {
	public static final DriveTrain kDriveTrain = new DriveTrain();
	public static final Gripper kGripper = new Gripper();
	public static final Elevator kElevator = new Elevator();
	// public static final Ramps kRamps = new Ramps();
	public static final RobotMap kRobotMap = new RobotMap();
	public static final Vision kVision = new Vision();
	public static final Dashboard kDashboard = new Dashboard();
	public static final Pneumatics kPneumatics = new Pneumatics();

	public static final LEDs kLEDs = new LEDs();

	public static OI m_oi;

	// public static RobotMap rMap;

	private static boolean m_autoStarted = false;
	private static double m_timeStart;
	public static String fieldMessage = "";

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		// rMap = new RobotMap();
		DriveTrain.motorSetup();
		Elevator.elevatorSetup();
		Vision.usbCamSetup();
		RobotMap.TAL_rightMaster.setSafetyEnabled(false);
		RobotMap.TAL_leftMaster.setSafetyEnabled(false);
		RobotMap.TAL_rightSlave.setSafetyEnabled(false);
		RobotMap.TAL_leftSlave.setSafetyEnabled(false);

		RobotMap.SLN_shiftingSolenoid.set(false);
		RobotMap.SLN_elevatorSolenoid.set(false);
		
		
		m_chooser.addDefault("Default", new AutonomousDefault());
		m_chooser.addObject("Right", new AutonomousRight());
		m_chooser.addObject("Center", new AutonomousCenter());
		m_chooser.addObject("Left", new AutonomousLeft());

		// m_chooser.addDefault("Default Auto", new testAuto());
		
		// chooser.addObject("My Auto", new MyAutoCommand());

		

	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		RobotMap.ahrs.reset();

		RobotMap.SLN_elevatorSolenoid.set(false);
		
		RobotMap.autoPin.set(true);
		
		Timer.delay(0.5);
		
		fieldMessage = DriverStation.getInstance().getGameSpecificMessage().toLowerCase();
		
		SmartDashboard.putString("FieldMessage", fieldMessage);
		
		/*m_chooser.addDefault("Default", new AutonomousDefault());
		m_chooser.addObject("Right", new AutonomousRight());
		m_chooser.addObject("Center", new AutonomousCenter());
		m_chooser.addObject("Left", new AutonomousLeft());
		
		SmartDashboard.putData("Auto mode", m_chooser);*/
		
		//m_autonomousCommand = m_chooser.getSelected();
		
		if (!RobotMap.left.get()) {
			m_autonomousCommand = new AutonomousLeft();
		}
		else if (!RobotMap.center.get()) {
			m_autonomousCommand = new AutonomousCenter();
		}
		else if (!RobotMap.right.get()) {
			m_autonomousCommand = new AutonomousRight();
		}
		else
		{
			m_autonomousCommand = new AutonomousDefault();
		}
		
		SmartDashboard.putString("Chosen Command", m_autonomousCommand.getName());
		
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
		 * ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		fieldMessage = DriverStation.getInstance().getGameSpecificMessage().toLowerCase();

		
		SmartDashboard.putNumber("IMU_Angle", RobotMap.ahrs.getAngle());
	}

	@Override
	public void teleopInit() {
		RobotMap.autoPin.set(false);
		
		RobotMap.TAL_rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
		RobotMap.TAL_leftMaster.getSensorCollection().setQuadraturePosition(0, 20);

		RobotMap.SLN_elevatorSolenoid.set(false);
		
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// RobotMap.compressor.setClosedLoopControl(true);
		// RobotMap.compressor.start();
		RobotMap.ahrs.reset();
		RobotMap.compressor.enabled();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

	}

	public WPI_TalonSRX TAL_carriageMotor = new WPI_TalonSRX(13);

	/**
	 * This function is called periodically during operator control.
	 */
	@SuppressWarnings("deprecation")
	@Override
	public void teleopPeriodic() {

		Scheduler.getInstance().run();
		
		if (RobotMap.sonar.getVoltage() < 0.3) {
			RobotMap.boxPin.set(true);
		} else {
			RobotMap.boxPin.set(false);
		}
		
		if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
			RobotMap.teamPin.set(true);
		} else {
			RobotMap.teamPin.set(false);
		}
		
		if (Timer.getMatchTime() < 30) {
			RobotMap.timePin.set(true);
		} else {
			RobotMap.timePin.set(false);
		}
		
		SmartDashboard.putNumber("Box Sensor Value", RobotMap.ALA_BoxSensor.getVoltage()); 
		SmartDashboard.putNumber("Elevator Position Percent", (RobotMap.TAL_elevatorMotor.getSensorCollection().getQuadraturePosition()/60));
		// SmartDashboard.putNumber("Elevator Pos2",
		// RobotMap.TAL_elevatorMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Elevator Setpoint Percent", (RobotMap.TAL_elevatorMotor.getClosedLoopTarget(0))/60);
		// SmartDashboard.putNumber("Elevator Current",
		// RobotMap.TAL_elevatorMotor.getOutputCurrent());
		SmartDashboard.putBoolean("The freaking limitswitch",
		RobotMap.TAL_elevatorMotor.getSensorCollection().isFwdLimitSwitchClosed());
		SmartDashboard.putNumber("IMU_Angle", RobotMap.ahrs.getAngle());
		SmartDashboard.putNumber("IMU_Yaw", RobotMap.ahrs.getYaw());
		SmartDashboard.putNumber("Voltage", DriverStation.getInstance().getBatteryVoltage());

		SmartDashboard.putNumber("Right Pos", RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Right Pos2",
		// RobotMap.TAL_rightMaster.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("Right Setpoint",
		// RobotMap.TAL_rightMaster.getClosedLoopTarget(0));

		SmartDashboard.putNumber("Left Pos", RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition());
		
		SmartDashboard.putNumber("Right Master Current", RobotMap.TAL_rightMaster.getOutputCurrent());
		SmartDashboard.putNumber("Left Master Current", RobotMap.TAL_leftMaster.getOutputCurrent());
		SmartDashboard.putNumber("Right Slave Current", RobotMap.TAL_rightSlave.getOutputCurrent());
		SmartDashboard.putNumber("Left Slave Current", RobotMap.TAL_leftSlave.getOutputCurrent());
				
		// SmartDashboard.putNumber("Left Pos2",
		// RobotMap.TAL_leftMaster.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("Left Setpoint",
		// RobotMap.TAL_leftMaster.getClosedLoopTarget(0));
		// RobotMap.currentTurnTicks =
		// ((RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition() +
		// RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition()) /2)/
		// RobotMap.encoder2TurnDegrees;
		SmartDashboard.putNumber("Turning Degrees", RobotMap.currentTurnDegrees);
		SmartDashboard.putNumber("Presurre sensor", ((RobotMap.ALG_PreSensor.getAverageVoltage() - 0.5222)/.0188));
		SmartDashboard.putBoolean("Left", RobotMap.left.get());
		SmartDashboard.putBoolean("Center", RobotMap.center.get());
		SmartDashboard.putBoolean("Right", RobotMap.right.get());
		SmartDashboard.putBoolean("DriverStop", RobotMap.driverStop);
		
		SmartDashboard.putNumber("Carriage Bus Voltage", RobotMap.TAL_carriageMotor.getBusVoltage());
		SmartDashboard.putNumber("Carriage Output Voltage", RobotMap.TAL_carriageMotor.getMotorOutputVoltage());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
