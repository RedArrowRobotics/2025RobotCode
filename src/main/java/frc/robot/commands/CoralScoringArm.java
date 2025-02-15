package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralScoringDeviceSubsystem;
import frc.robot.subsystems.CoralScoringDeviceSubsystem.CoralArmPosition;

public class CoralScoringArm extends PIDCommand {
    private CoralScoringDeviceSubsystem subsystem;
    private CoralArmPosition target;

    public CoralScoringArm(CoralScoringDeviceSubsystem subsystem, CoralArmPosition target) {
        super(0.5, 0.0, 0.0, 
        subsystem.scorerTilter, 
        switch(target) {
            case HOME -> Constants.scorerTilterLoadingPosition;
            case L2L3 -> Constants.scorerTilterScoringPositionL2L3;
            case L4 -> Constants.scorerTilterScoringPositionL4;
            case NONE -> Constants.scorerTilterLoadingPosition;
         }, 
         10.0);
        this.target = target;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        subsystem.coralArmPosition = CoralArmPosition.NONE;
    }

    @Override
    public void finalize() {
        super.finalize();
        subsystem.coralArmPosition = target;
    }
}
