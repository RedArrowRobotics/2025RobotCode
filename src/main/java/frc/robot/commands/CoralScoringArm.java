package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.CoralScoringDeviceSubsystem;
import frc.robot.subsystems.CoralScoringDeviceSubsystem.CoralArmPosition;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CoralScoringArm extends PIDCommand {
    private CoralScoringDeviceSubsystem subsystem;
    private CoralArmPosition target;

    public CoralScoringArm(CoralScoringDeviceSubsystem subsystem, CoralArmPosition target) {
        super(0.1, 0.0, 0.0, 
        subsystem.scorerTilter, 
        switch(target) {
            case HOME -> Constants.scorerTilterLoadingPosition;
            case L2L3 -> Constants.scorerTilterScoringPositionL2L3;
            case L4 -> Constants.scorerTilterScoringPositionL4;
            case NONE -> Constants.scorerTilterLoadingPosition;
         }, 
         1.0,
         true);
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

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.addBooleanProperty("Is Finished", () -> isFinished(), null);
    }
}
