package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CageSubsystem extends SubsystemBase {
SparkMax cageGrabber = new SparkMax(Constants.cageGrabberMotorId, MotorType.kBrushed);
SparkMax cageClimber = new SparkMax(Constants.cageClimberMotorId, MotorType.kBrushed);
/**
 *  Rotates a bar to hold the cage in place for climbing.
 */
  public Command holdCage() {
    return startEnd(
        () -> {
          cageGrabber.set(.5);
        },
        () -> {
          cageGrabber.set(0);
        }
        ).until(() -> cageGrabber.getEncoder().equals(Constants.cageGrabberHoldPosition));
        //getEncoder or getPosition?
  }

/**
 *  Rotates the bar back to the starting position so the cage is free to move.
 */
  public Command releaseCage() {
    return startEnd(
        () -> {
          cageGrabber.set(-.5);
        },
        () -> {
          cageGrabber.set(0);
        }
        ).until(() -> cageGrabber.getEncoder().equals(Constants.cageGrabberReleasePosition));
        //getEncoder or getPosition?
  }

  /**
   *  Pushes down on the cage to raise the robot.
 */
  public Command ascend() {
    return startEnd(
      () -> {
        cageClimber.set(1);
      },
      () -> {
        cageClimber.set(0);
      }
    );
  }

  /**
 *  Retracts the climber to lower the robot.
 */
  public Command descend() {
    return startEnd(
      () -> {
        cageClimber.set(-1);
      },
      () -> {
        cageClimber.set(0);
      }
    );
  }

    /**
     *  Checks if the cage is in the holding position.
     */
  public boolean isCageHeld() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
