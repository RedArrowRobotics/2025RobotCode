package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CageSubsystem extends SubsystemBase {

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
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
 *  Retracts the climber to lower the robot.
 */
public Command descend() {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
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
