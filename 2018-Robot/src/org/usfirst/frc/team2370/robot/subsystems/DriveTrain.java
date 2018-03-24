/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.subsystems;

import org.usfirst.frc.team2370.robot.Robot;
import org.usfirst.frc.team2370.robot.RobotMap;
import org.usfirst.frc.team2370.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Drive Train Subsystem
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	/**
	 * Method to setup the slave speed controllers to follower mode
	 */

	static double speed = .9;
	static double turnSpeed = 0.50;

	public static void motorSetup() {
		int timeout = 1000;

		RobotMap.TAL_leftSlave.follow(RobotMap.TAL_leftMaster);
		RobotMap.TAL_rightSlave.follow(RobotMap.TAL_rightMaster);

		RobotMap.TAL_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout);
		RobotMap.TAL_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, timeout);
		RobotMap.TAL_leftMaster.getSensorCollection().setQuadraturePosition(0, timeout);
		RobotMap.TAL_rightMaster.getSensorCollection().setQuadraturePosition(0, timeout);

		/*
		 * RobotMap.TAL_rightMaster.setInverted(true);
		 * RobotMap.TAL_leftMaster.setInverted(true);
		 * RobotMap.TAL_rightSlave.setInverted(true);
		 * RobotMap.TAL_leftSlave.setInverted(true);
		 */

		try {
			RobotMap.ahrs = new AHRS(SerialPort.Port.kUSB);// , AHRS.SerialDataType.kRawData, RobotMap.updateRate);//
															// kOnboard);
			RobotMap.ahrs.enableLogging(true);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		RobotMap.ahrs.reset();

	}

	/**
	 * Use arcade drive from the driveTrain using a speed and rotation parameter
	 * 
	 * @param speed
	 *            - The speed (forward and backward) of the speed controllers (-1.0
	 *            to 1.0)
	 * @param rotation
	 *            - The rotation (left and right) of the speed controllers (-1.0 to
	 *            1.0)
	 */
	public static void arcadeDrive(double speed, double rotation) {
		RobotMap.driveTrain.arcadeDrive(speed, rotation);
	}

	/**
	 * A method to stop all drive motors for use at the end of some commands
	 */
	public static void stopMotors() {
		RobotMap.TAL_rightMaster.set(0);
		RobotMap.TAL_leftMaster.set(0);
	}

	/**
	 * A method to turn the robot a specific number of degrees based on
	 * accelerometer
	 * 
	 * @param angle
	 *            The angle the robot will turn (Only positive angles)
	 */
	public static void turnRight(double angle) {
		/*
		 * RobotMap.currentTurnDegrees =
		 * ((RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition() +
		 * RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition())
		 * /2)/RobotMap.encoder2TurnDegrees; if (RobotMap.currentTurnDegrees > angle*-1)
		 * { RobotMap.TAL_rightMaster.set(turnSpeed);
		 * RobotMap.TAL_leftMaster.set(turnSpeed); } else {
		 * RobotMap.TAL_rightMaster.set(0); RobotMap.TAL_leftMaster.set(0); }
		 */
		if (RobotMap.ahrs.getAngle() < RobotMap.oldAngle + angle) {
			RobotMap.TAL_rightMaster.set(turnSpeed );
			RobotMap.TAL_leftMaster.set(turnSpeed );
		}
		// } else if (RobotMap.ahrs.getAngle() > RobotMap.oldAngle + angle) {
		// RobotMap.TAL_rightMaster.set(-turnSpeed / 3);
		// RobotMap.TAL_leftMaster.set(-turnSpeed / 3);
		else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
		// Robot.kDriveTrain.getPIDController().setSetpoint(angle);
	}

	/**
	 * A method to turn the robot a specific number of degrees based on
	 * accelerometer
	 * 
	 * @param angle
	 *            The angle the robot will turn (Only positive angles)
	 */
	public static void turnLeft(double angle) {
		/*
		 * RobotMap.currentTurnDegrees =
		 * ((RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition() +
		 * RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition())
		 * /2)/RobotMap.encoder2TurnDegrees; if (RobotMap.currentTurnDegrees < angle) {
		 * RobotMap.TAL_rightMaster.set(-turnSpeed);
		 * RobotMap.TAL_leftMaster.set(-turnSpeed); } else {
		 * RobotMap.TAL_rightMaster.set(0); RobotMap.TAL_leftMaster.set(0); }
		 */
		/*
		 * if (RobotMap.ahrs.getAngle() >= RobotMap.oldAngle - angle + error) {
		 * RobotMap.TAL_rightMaster.set(turnSpeed*-1);
		 * RobotMap.TAL_leftMaster.set(turnSpeed*-1);
		 */
		if (RobotMap.ahrs.getAngle() > RobotMap.oldAngle - angle) {
			RobotMap.TAL_rightMaster.set((-1 * turnSpeed));
			RobotMap.TAL_leftMaster.set((-1 * turnSpeed));
		}
		// } else if (RobotMap.ahrs.getAngle() < RobotMap.oldAngle - angle) {
		// RobotMap.TAL_rightMaster.set(turnSpeed / 3);
		// RobotMap.TAL_leftMaster.set(turnSpeed / 3);
		else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
		// Robot.kDriveTrain.getPIDController().setSetpoint(angle*-1);
	}

	/**
	 * A method to turn the robot to a specific angle based on accelerometer
	 * 
	 * @param angle
	 *            The angle the robot will turn to in degrees
	 */
	public static void turnFieldOriented(double angle) {

	}

	/**
	 * A method to drive the robot forward a specific distance (In inches) using
	 * encoders
	 * 
	 * @param distance
	 *            The distance (In inches) to drive forward
	 */
	public static void driveForward(double distance) {
		double rightEnc = RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition();
		double leftEnc = RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition();
		double fixedDistance = distance * RobotMap.encoder2actual;
		double error = rightEnc / fixedDistance;

		// Angle matching
		if ((rightEnc < fixedDistance) || leftEnc > fixedDistance * -1) {

			if (error > 0.5 && error < 0.7) {
				// Veering left, correct towards the right
				if (RobotMap.originalAngle > RobotMap.ahrs.getAngle()) {
					RobotMap.TAL_rightMaster.set((-1 * speed) / 1.25);
					RobotMap.TAL_leftMaster.set((speed + 0.1) / 1.25);
				}

				// Veering right, correct towards the left
				else if (RobotMap.originalAngle < RobotMap.ahrs.getAngle()) {
					RobotMap.TAL_rightMaster.set((-1 * speed - 0.1) / 1.25);
					RobotMap.TAL_leftMaster.set(speed / 1.25);
				}

				// Right on the money
				else {
					RobotMap.TAL_rightMaster.set((-1 * speed) / 1.25);
					RobotMap.TAL_leftMaster.set(speed / 1.25);
				}

			}
			if (error > 0.7) {
				// Veering left, correct towards the right
				if (RobotMap.originalAngle > RobotMap.ahrs.getAngle()) {
					RobotMap.TAL_rightMaster.set((-1 * speed) / 1.5);
					RobotMap.TAL_leftMaster.set((speed + 0.1) / 1.5);
				}

				// Veering right, correct towards the left
				else if (RobotMap.originalAngle < RobotMap.ahrs.getAngle()) {
					RobotMap.TAL_rightMaster.set((-1 * speed - 0.1) / 1.5);
					RobotMap.TAL_leftMaster.set(speed / 1.5);
				}

				// Right on the money
				else {
					RobotMap.TAL_rightMaster.set((-1 * speed) / 1.5);
					RobotMap.TAL_leftMaster.set(speed / 1.5);
				}

			} else {
				// Veering left, correct towards the right
				if (RobotMap.originalAngle > RobotMap.ahrs.getAngle()) {
					RobotMap.TAL_rightMaster.set(-1 * speed);
					RobotMap.TAL_leftMaster.set(speed + 0.1);
				}

				// Veering right, correct towards the left
				else if (RobotMap.originalAngle < RobotMap.ahrs.getAngle()) {
					RobotMap.TAL_rightMaster.set(-1 * speed - 0.1);
					RobotMap.TAL_leftMaster.set(speed);
				}

				// Right on the money
				else {
					RobotMap.TAL_rightMaster.set(-1 * speed);
					RobotMap.TAL_leftMaster.set(speed);
				}
			}

			// Stop the motors, you've made it to your destination
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}

		// Encoder matching
		/*
		 * if ((RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() <
		 * distance * RobotMap.encoder2actual ) ||
		 * RobotMap.TAL_leftMaster.getSensorCollection() .getQuadraturePosition() >
		 * (distance * RobotMap.encoder2actual) * -1) {
		 * 
		 * if (RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() >
		 * (RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition()*-1)) {
		 * 
		 * RobotMap.TAL_rightMaster.set(-1 * speed); RobotMap.TAL_leftMaster.set(speed +
		 * 0.3); }
		 * 
		 * if (RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() <
		 * (RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition()*-1)) {
		 * 
		 * RobotMap.TAL_rightMaster.set(-1 * speed - 0.3);
		 * RobotMap.TAL_leftMaster.set(speed); } else { RobotMap.TAL_rightMaster.set(-1
		 * * speed); RobotMap.TAL_leftMaster.set(speed); }
		 * 
		 * } else { RobotMap.TAL_rightMaster.set(0); RobotMap.TAL_leftMaster.set(0); }
		 */

	}

	public static void driveForwardSlow(double distance) {
		double rightEnc = RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition();
		double leftEnc = RobotMap.TAL_leftMaster.getSensorCollection().getQuadraturePosition();
		double fixedDistance = distance * RobotMap.encoder2actual;
		double error = rightEnc / fixedDistance;

		// Angle matching
		if ((RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() < distance
				* RobotMap.encoder2actual)
				&& RobotMap.TAL_leftMaster.getSensorCollection()
						.getQuadraturePosition() > (distance * RobotMap.encoder2actual) * -1) {
			RobotMap.TAL_rightMaster.set((-1 * speed) / 2.25);
			RobotMap.TAL_leftMaster.set(speed / 2.25);
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
	}

	public static void driveBackwardsSlow(double distance) {
		if ((RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() > -distance
				* RobotMap.encoder2actual)
				&& RobotMap.TAL_leftMaster.getSensorCollection()
						.getQuadraturePosition() < (-distance * RobotMap.encoder2actual) * -1) {
			RobotMap.TAL_rightMaster.set((-1 * -speed) / 2.5);
			RobotMap.TAL_leftMaster.set(-speed / 2.5);
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
	}

	public static void driveBackwards(double distance) {
		if ((RobotMap.TAL_rightMaster.getSensorCollection().getQuadraturePosition() > -distance
				* RobotMap.encoder2actual)
				&& RobotMap.TAL_leftMaster.getSensorCollection()
						.getQuadraturePosition() < (-distance * RobotMap.encoder2actual) * -1) {
			RobotMap.TAL_rightMaster.set(-1 * -speed);
			RobotMap.TAL_leftMaster.set(-speed);
		} else {
			RobotMap.TAL_rightMaster.set(0);
			RobotMap.TAL_leftMaster.set(0);
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new DriveWithJoystick());
	}

	public static void driveRight() {
		RobotMap.TAL_rightMaster.set((-1 * speed) + .255);
		RobotMap.TAL_leftMaster.set(speed + .08);

	}

	public static void driveLeft() {
		RobotMap.TAL_rightMaster.set((-1 * speed - 0.07));
		RobotMap.TAL_leftMaster.set(speed - .20);

	}

}
