package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralScoringDeviceSubsystem extends SubsystemBase {

  /**
   * It securely grabs the coral that fell from the chute.
   */
  public Command grabCoral() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Drops the coral onto the reef.
   */
  public Command dropCoral() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Stops the coral scorer's wheels from spining.
   */
  public Command stopScorerSpin() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
 /**
   * Moves the coral scorer to the correct angle to drop it on the reef.
   */
  public Command placeCoralPosition() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
 /**
   * Moves the coral scorer to the correct angle to grab the coral that fell from the chute.
   */
  public Command loadCoralPosition() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

   /**
   * Checks to see if the coral is correctly loaded on the coral scorer.
   */
  public boolean isCoralLoaded() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

   /**
   * Checks to see if the coral scorer is aligned with the reef.
   */
  public boolean isCoralOverReef() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
