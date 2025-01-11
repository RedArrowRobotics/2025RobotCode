package frc.robot.auto;

import frc.robot.SwerveDriveTrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

public class AutoMove {

    //Variable Defintions

    private Pose2d startPose;
    private Pose2d endPose;
    private long movementDurationInNanoSeconds;
    private long startTime;
    
    public AutoMove (Pose2d startPose2d, Pose2d endPose2d, double moveTimeInSeconds) {
        startPose = startPose2d;
        endPose = endPose2d;
        movementDurationInNanoSeconds = (long)(moveTimeInSeconds * 1000000000);        
    }

    public final void moveInit()
    {
        startTime = System.nanoTime();
    }

    public final boolean moveExecute(SwerveDriveTrain swerveDriveTrain) {
        long currentTimeInNano = System.nanoTime();
        long elapsedTime = currentTimeInNano - startTime;
        Pose2d targetPose = startPose.interpolate(endPose, 
            (double)elapsedTime/(double)movementDurationInNanoSeconds);
        Pose2d currentPose = swerveDriveTrain.getPose();

        Twist2d twist = currentPose.log(targetPose);

        swerveDriveTrain.driveFC(twist.dx, twist.dy, twist.dtheta);

        return (elapsedTime > movementDurationInNanoSeconds);
    }
}