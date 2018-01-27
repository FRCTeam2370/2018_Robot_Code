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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team2370.robot.commands.ExampleDriveCommand;
import org.usfirst.frc.team2370.robot.subsystems.Dashboard;
import org.usfirst.frc.team2370.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2370.robot.subsystems.Elevator;
import org.usfirst.frc.team2370.robot.subsystems.Gripper;
import org.usfirst.frc.team2370.robot.subsystems.LEDs;
import org.usfirst.frc.team2370.robot.subsystems.Ramps;
import org.usfirst.frc.team2370.robot.subsystems.Vision;

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
	public static final Ramps kRamps = new Ramps();
	public static final Vision kVision = new Vision();
	public static final Dashboard kDashboard = new Dashboard();


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
		// rMap = new RobotMap();
		DriveTrain.motorSetup();
		Vision.usbCamSetup();
		m_chooser.addDefault("Default Auto", new ExampleDriveCommand());
		// chooser.addObject("My Auto", new MyAutoCommand());
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
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("Position", RobotMap.elevatorMotor.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Position2", RobotMap.elevatorMotor.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Velocity", RobotMap.elevatorMotor.getSensorCollection().getQuadratureVelocity());
		SmartDashboard.putNumber("Error", RobotMap.elevatorMotor.getErrorDerivative(0));
		SmartDashboard.putNumber("Error2", RobotMap.elevatorMotor.getClosedLoopError(0));
		SmartDashboard.putNumber("Setpoint", RobotMap.elevatorMotor.getClosedLoopTarget(0));
	
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
