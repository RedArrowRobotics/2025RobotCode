package frc.robot.pid;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public record ElevatorFeedforwardParameters(double Ks, double Kg, double Kv) {
    public ElevatorFeedforward createController() {
        return new ElevatorFeedforward(Ks, Kg, Kv);
    }
}