package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  /**
   * Moves the elevator to the home position which is the lowest position.
   */
  public Command elevatorHome() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * Moves the elevator to the L2 position.
   */
  public Command elevatorL2() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Moves the elevator to the L3 position.
   */
  public Command elevatorL3() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Moves the elevator to the L4 position.
   */
  public Command elevatorL4() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Moves the dealgaer into the dealgae position.
   */
  public Command dealgaeExtend() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Moves the dealgaer into the stored position.
   */
  public Command dealgaeRetract() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Activates the dealgaer.
   */
  public Command dealgaeStartSpin() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * Deactivates the dealgaer.
   */
  public Command dealgaeStopSpin() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
}
