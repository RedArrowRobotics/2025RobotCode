package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SensorInputs;

public class CoralScoringDeviceSubsystem extends SubsystemBase {
  SparkMax intakeWheels = new SparkMax(Constants.intakeWheelsID, MotorType.kBrushed);
  SparkMax scorerTilter = new SparkMax(Constants.coralScorerTilterID, MotorType.kBrushed);
  private DigitalInput coralSensor = new DigitalInput(Constants.coralSensorChannel);
  private DigitalInput reefSensor = new DigitalInput(Constants.reefSensorChannel);

  /**
   * It securely grabs the coral that fell from the chute.
   * Sucks the coral down with wheels to hold it in place.
   */
  public Command grabCoral() {
    return Commands.startEnd(
      () -> {
        intakeWheels.set(1);
      },
      () -> {
        intakeWheels.set(0);
      }
    ).until(() -> isCoralLoaded());
  }
  
  /**
   * Drops the coral onto the reef.
   */
  public Command dropCoral() {
    return Commands.startEnd(
      () -> {
        intakeWheels.set(-1);
      },
      () -> {
        intakeWheels.set(0);
      }
    );
  }
  
  /**
   * Stops the coral scorer's wheels from spining.
   */
  public Command stopScorerSpin() {
    return runOnce(
        () -> {
          intakeWheels.set(0);
        });
  }
  
 /**
   * Moves the coral scorer to the correct angle to drop it on the reef.
   */
  public Command placeCoralPosition() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
 /**
   * Moves the coral scorer to the correct angle to grab the coral that fell from the chute.
   */
  public Command loadCoralPosition() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

   /**
   * Checks to see if the coral is correctly loaded on the coral scorer.
   */
  public boolean isCoralLoaded() {
    return coralSensor.get();
  }

   /**
   * Checks to see if the coral scorer is aligned with the reef.
   */
  public boolean isCoralOverReef() {
    return reefSensor.get();
  }
}
