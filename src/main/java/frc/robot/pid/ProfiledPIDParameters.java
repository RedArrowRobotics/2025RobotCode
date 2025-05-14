package frc.robot.pid;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record ProfiledPIDParameters(double Kp, double Ki, double Kd, double maxVelocity, double maxAcceleration) {
    public ProfiledPIDController createController() {
        return new ProfiledPIDController(Kp, Ki, Kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }
}