package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class PIDCommand extends Command {
    private PIDController pidController;
    private SparkMax motor;
    private double target;
    private double tolerance;
    private boolean invertMotor;

    public PIDCommand(double kp, double ki, double kd, SparkMax motorController, double targetPosition, double encoderTolerance, boolean invertMotor) {
        pidController = new PIDController(kp, ki, kd);
        motor = motorController;
        target = targetPosition;
        tolerance = encoderTolerance;
        this.invertMotor = invertMotor;
    }

    @Override
    public void initialize() {
        //Get the motor controller and target position
    }

    @Override
    public void execute() {
        //Get current position, adjust motor speed, and set speed
        double power;
        if(invertMotor == true) {
            power = pidController.calculate(motor.getEncoder().getPosition(), target) * -1.0;
        } else {
            power = pidController.calculate(motor.getEncoder().getPosition(), target);
        }
        motor.set(power);
    }

    @Override
    public boolean isFinished() {
        boolean finished = Math.abs(motor.getEncoder().getPosition() - target) < tolerance;
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0.0);
    }
}

