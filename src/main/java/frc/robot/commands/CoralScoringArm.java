package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralScoringDeviceSubsystem;

public class CoralScoringArm extends PIDCommand {
    public CoralScoringArm(CoralScoringDeviceSubsystem subsystem, double position) {
        super(0.0, 0.0, 0.0, subsystem.scorerTilter, position, 10.0);
        addRequirements(subsystem);
    }
}
