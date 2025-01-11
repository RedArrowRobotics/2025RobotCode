package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Components;
import frc.robot.SwerveDriveTrain;
import frc.robot.SensorInputs;

//Recommended tolerance in IN is 2.0

public class MoveInlineAutoAction implements AutoAction {
    private AutoMove auto;

    public MoveInlineAutoAction(Pose2d start, Pose2d end, double moveTimeInSeconds)
    {
        auto = new AutoMove(start,end,moveTimeInSeconds);
    }

    //Time in seconds to execute the move
    private double maxTime;

    //Distance of move in inches
    private double distance;
    
    //Allowable tolerance (error) of the error in encoder counts
    private double tolerance;

    //Scalar constant applied to the power output
    private double moveK = 0.125;

    @Override
    public void init(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor) {
        auto.moveInit();
    }

    @Override
    public boolean execute(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor) {
        return auto.moveExecute(swerveDriveTrain);
    }

    @Override
    public void finalize(SwerveDriveTrain swerveDriveTrain, Components components, SensorInputs sensor) {}

    @Override
    public String toString() {
        return "Auto: Move Inline";
    }
}