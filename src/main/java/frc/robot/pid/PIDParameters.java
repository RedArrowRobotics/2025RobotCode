package frc.robot.pid;

import edu.wpi.first.math.controller.PIDController;

public record PIDParameters(double Kp, double Ki, double Kd) {
    public PIDController createController() {
        return new PIDController(Kp, Ki, Kd);
    }
}