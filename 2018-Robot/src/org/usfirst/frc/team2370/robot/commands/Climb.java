package org.usfirst.frc.team2370.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Climb extends CommandGroup {

    public Climb() {
        addSequential(new ElevatorToClimb());
        addSequential(new CarriageToTop());
    }
}
