package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class PIDCommand extends Command {
    private PIDController pidController;
    private SparkMax motor;
    private double target;
    private double tolerance;

    public PIDCommand(double kp, double ki, double kd, SparkMax motorController, double targetPosition, double encoderTolerance) {
        pidController = new PIDController(kp, ki, kd);
        motor = motorController;
        target = targetPosition;
        tolerance = encoderTolerance;
    }

    @Override
    public void initialize() {
        //Get the motor controller and target position
    }

    @Override
    public void execute() {
        //Get current position, adjust motor speed, and set speed
        motor.set(pidController.calculate(motor.getEncoder().getPosition(), target));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(motor.getEncoder().getPosition() - target) < tolerance;
    }

    @Override
    public void finalize() {
        //Set speed to zero
        motor.set(0.0);
    }
}
