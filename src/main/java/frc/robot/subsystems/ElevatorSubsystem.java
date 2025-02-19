package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
SparkMax elevatorMotor = new SparkMax(Constants.elevatorMotorId, MotorType.kBrushed);
SparkMax dealgaeFlipper = new SparkMax(Constants.deAlgaeFlipperId, MotorType.kBrushed);
SparkMax dealgaeWheels = new SparkMax(Constants.deAlgaeWheelsId, MotorType.kBrushed);
PIDController elevatorPID = new PIDController(0.1, 0, 0);

  /**
   * Moves the elevator to the home position which is the lowest position.
   */
  public Command elevatorHome() {
    return runOnce(
        () -> {
          elevatorMotor.set(1);
        });
  }

  /**
   * Moves the elevator to the L2 position.
   */
  public Command elevatorL2() {
    return runOnce(
        () -> {
          elevatorMotor.set(1);
        });
  }
  
  /**
   * Moves the elevator to the L3 position.
   */
  public Command elevatorL3() {
    return runOnce(
        () -> {
          elevatorMotor.set(1);
        });
  }
  
  /**
   * Moves the elevator to the L4 position.
   */
  public Command elevatorL4() {
    return runOnce(
        () -> {
          elevatorMotor.set(1);
        });
  }
  
  /**
   * Moves the dealgaer into the dealgae position.
   */
  public Command dealgaeExtend() {
    return runOnce(
        () -> {
          dealgaeFlipper.set(1);
        });
  }
  
  /**
   * Moves the dealgaer into the stored position.
   */
  public Command dealgaeRetract() {
    return runOnce(
        () -> {
          dealgaeFlipper.set(1);
        });
  }
  
  /**
   * Activates the dealgaer.
   */
  public Command dealgaeStartSpin() {
    return runOnce(
        () -> {
          dealgaeWheels.set(1);
        });
  }
  
  /**
   * Deactivates the dealgaer.
   */
  public Command dealgaeStopSpin() {
    return runOnce(
        () -> {
          dealgaeWheels.set(0);
        });
  }
}
