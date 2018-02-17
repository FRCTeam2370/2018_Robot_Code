/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2370.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * An example command. You can replace me with your own command.
 */
public class testAuto extends CommandGroup {
	public testAuto() {
		addSequential(new TurnRight(90));//, 10);
	}
}
