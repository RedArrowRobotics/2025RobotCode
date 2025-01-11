package frc.robot.auto;

import frc.robot.Components;
import frc.robot.SwerveDriveTrain;
import frc.robot.SensorInputs;

public class DoNothingAutoAction implements AutoAction {
    @Override
    public void init(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor) {}
    
    @Override
    public boolean execute(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensors) {
        // driveTrain.mecanumDrive(0, 0, 0, driveTrain.defaultRotation2d);
        swerveDriveTrain.driveFC(0.0, 0.0, 0.0);
        return false;
    }

    @Override
    public void finalize(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor) {}

    @Override
    public String toString() {
        return "Auto: Do Nothing";
    }
}