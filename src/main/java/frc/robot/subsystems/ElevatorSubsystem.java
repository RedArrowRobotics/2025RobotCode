package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
SparkMax elevatorMotor1 = new SparkMax(Constants.elevatorMotor1Id, MotorType.kBrushed);
SparkMax elevatorMotor2 = new SparkMax(Constants.elevatorMotor2Id, MotorType.kBrushed);
SparkMax dealgaeFlipper = new SparkMax(Constants.dealgaeFlipperId, MotorType.kBrushed);
SparkMax dealgaeWheels = new SparkMax(Constants.dealgaeWheelsId, MotorType.kBrushed);
PIDController elevatorPID = new PIDController(0.1, 0, 0);

  /**
   * Moves the elevator to the home position which is the lowest position.
   */
  public Command elevatorHome() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> elevatorMotor1.getEncoder().equals(Constants.elevatorMotorHomePosition));
        //getEncoder or getPosition?
  }

  /**
   * Moves the elevator to the L2 position.
   */
  public Command elevatorL2() {
      return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> elevatorMotor1.getEncoder().equals(Constants.elevatorMotorL2Position));
        //getEncoder or getPosition?
  }
  
  /**
   * Moves the elevator to the L3 position.
   */
  public Command elevatorL3() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> elevatorMotor1.getEncoder().equals(Constants.elevatorMotorL3Position));
        //getEncoder or getPosition?
  }
  
  /**
   * Moves the elevator to the L4 position.
   */
  public Command elevatorL4() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> elevatorMotor1.getEncoder().equals(Constants.elevatorMotorL4Position));
        //getEncoder or getPosition?
  }
  
  /**
   * Moves the dealgaer into the dealgae position.
   */
  public Command dealgaeExtend() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> dealgaeFlipper.getEncoder().equals(Constants.dealgaeFlipperExtendedPosition));
        //getEncoder or getPosition?
  }
  
  /**
   * Moves the dealgaer into the stored position.
   */
  public Command dealgaeRetract() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> dealgaeFlipper.getEncoder().equals(Constants.dealgaeFlipperRetractedPosition));
        //getEncoder or getPosition?
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
