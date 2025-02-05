package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageSubsystem extends SubsystemBase {
SparkMax cageGrabber = new SparkMax(13, MotorType.kBrushed);
SparkMax cageClimber = new SparkMax(13, MotorType.kBrushed);
/**
 *  Rotates a bar to hold the cage in place for climbing.
 */
public Command holdCage() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

/**
 *  Rotates the bar back to the starting position so the cage is free to move.
 */
public Command releaseCage() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
 *  Pushes down on the cage to raise the robot.
 */
  public Command ascend() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
 *  Retracts the climber to lower the robot.
 */
public Command descend() {
  return runOnce(
      () -> {
        /* one-time action goes here */
      });
}

    /**
     *  Checks if the cage is in the holding position.
     */
  public boolean isCageHeld() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
