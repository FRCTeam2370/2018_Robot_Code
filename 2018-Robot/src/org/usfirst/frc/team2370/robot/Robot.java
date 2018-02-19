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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2370.robot.commands.AutonomousLeft;
import org.usfirst.frc.team2370.robot.commands.AutonomousRight;
import org.usfirst.frc.team2370.robot.commands.testAuto;
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
	//public static final Ramps kRamps = new Ramps();
	public static final RobotMap kRobotMap = new RobotMap();
	public static final Vision kVision = new Vision();
	public static final Dashboard kDashboard = new Dashboard();
	public static final Pneumatics kPneumatics = new Pneumatics();

	public static final LEDs kLEDs = new LEDs();

	public static OI m_oi;
	
	
	// public static RobotMap rMap;

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		RobotMap.fieldMessage = "rrr";
		// rMap = new RobotMap();
		DriveTrain.motorSetup();
		Elevator.elevatorSetup();
		Vision.usbCamSetup();
		RobotMap.SLN_shiftingSolenoid.set(false);
		RobotMap.SLN_elevatorSolenoid.set(false);
		m_chooser.addDefault("Default Auto", new testAuto());
		m_chooser.addObject("Auto Right", new AutonomousRight());
		m_chooser.addObject("Autonomous Left", new AutonomousLeft());
		SmartDashboard.putData("Auto mode", m_chooser);
		
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
		kRobotMap.fieldMessage = DriverStation.getInstance().getGameSpecificMessage().toLowerCase();
		m_autonomousCommand = m_chooser.getSelected();
		
		

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
		 * ExampleCommand(); break; }
		 */
		

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("IMU_Angle", RobotMap.ahrs.getAngle());
	}

	@Override
	public void teleopInit() {
		
		RobotMap.TAL_rightMaster.getSensorCollection().setQuadraturePosition(0, 20);
		RobotMap.TAL_leftMaster.getSensorCollection().setQuadraturePosition(0, 20);
		
		
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		// RobotMap.compressor.setClosedLoopControl(true);
		// RobotMap.compressor.start();
		RobotMap.compressor.enabled();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		
	}

	public WPI_TalonSRX TAL_carriageMotor = new WPI_TalonSRX(13);
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		
		// SmartDashboard.putNumber("Position",
		// RobotMap.elevatorMotor.getSensorCollection().getQuadraturePosition());
		// SmartDashboard.putNumber("Position2",
		// RobotMap.elevatorMotor.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("Velocity",
		// RobotMap.elevatorMotor.getSensorCollection().getQuadratureVelocity());
		// SmartDashboard.putNumber("Error",
		// RobotMap.elevatorMotor.getErrorDerivative(0));
		// SmartDashboard.putNumber("Error2",
		// RobotMap.elevatorMotor.getClosedLoopError(0));
		// SmartDashboard.putNumber("Setpoint",
		// RobotMap.elevatorMotor.getClosedLoopTarget(0));
//		SmartDashboard.putNumber("Voltage", RobotMap.TAL_leftMaster.getMotorOutputVoltage());
//		SmartDashboard.putNumber("IMU_Yaw", RobotMap.ahrs.getYaw());
//		SmartDashboard.putNumber("IMU_Pitch", RobotMap.ahrs.getPitch());
//		SmartDashboard.putNumber("IMU_Roll", RobotMap.ahrs.getRoll());
		SmartDashboard.putNumber("IMU_Angle", RobotMap.ahrs.getAngle());
//		SmartDashboard.putNumber("Pres Sensor", RobotMap.ALA_PreSensor.getValue());
		
		/*SmartDashboard.putNumber("Position Left", RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Velocity Left", RobotMap.TAL_leftMaster.getSensorCollection().getQuadratureVelocity());
		SmartDashboard.putNumber("Error Left", RobotMap.TAL_leftMaster.getClosedLoopError(0));
		SmartDashboard.putNumber("Position Right", RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Velocity Right", RobotMap.TAL_rightMaster.getSensorCollection().getQuadratureVelocity());
		SmartDashboard.putNumber("Error Right", RobotMap.TAL_rightMaster.getClosedLoopError(0));*/
//		SmartDashboard.putNumber("Elevator Pos", RobotMap.TAL_elevatorMotor.getSensorCollection().getQuadraturePosition());
//		SmartDashboard.putNumber("Elevator Pos2", RobotMap.TAL_elevatorMotor.getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("Elevator Setpoint", RobotMap.TAL_elevatorMotor.getClosedLoopTarget(0));
//		SmartDashboard.putNumber("Elevator Current", RobotMap.TAL_elevatorMotor.getOutputCurrent());
		
		
		
		SmartDashboard.putNumber("Right Pos", RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition());
//		SmartDashboard.putNumber("Right Pos2", RobotMap.TAL_rightMaster.getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("Right Setpoint", RobotMap.TAL_rightMaster.getClosedLoopTarget(0));
		
		SmartDashboard.putNumber("Left Pos", RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition());
//		SmartDashboard.putNumber("Left Pos2", RobotMap.TAL_leftMaster.getSelectedSensorPosition(0));
//		SmartDashboard.putNumber("Left Setpoint", RobotMap.TAL_leftMaster.getClosedLoopTarget(0));
		 
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
