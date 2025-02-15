package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class CoralScoringDeviceSubsystem extends SubsystemBase {
  SparkMax intakeWheels = new SparkMax(Constants.intakeWheelsMotorID, MotorType.kBrushed);
  SparkMax scorerTilter = new SparkMax(Constants.coralScorerTilterMotorID, MotorType.kBrushed);
  private DigitalInput coralSensor = new DigitalInput(Constants.coralSensorChannel);
  private DigitalInput reefSensor = new DigitalInput(Constants.reefSensorChannel);
  public final Trigger reefTrigger = new Trigger(reefSensor::get);
  /**
   * It securely grabs the coral that fell from the chute.
   * Sucks the coral down with wheels to hold it in place.
   */
  public Command grabCoral() {
    return startEnd(
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
    return startEnd(
      () -> {
        intakeWheels.set(-1);
      },
      () -> {
        intakeWheels.set(0);
      }
    ).withTimeout(Constants.coralDropTimeout);
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
    return startEnd(
        () -> {
          scorerTilter.set(.5);
          /* one-time action goes here */
        },
        () -> {
          scorerTilter.set(0);
        }
        ).until(() -> scorerTilter.getEncoder().equals(Constants.scorerTilterScoringPosition));
        //getEncoder or getPosition?
  }
  
 /**
   * Moves the coral scorer to the correct angle to grab the coral that fell from the chute.
   */
  public Command loadCoralPosition() {
    return startEnd(
        () -> {
          if(scorerTilter.getEncoder().getPosition() > Constants.scorerTilterLoadingPosition) {
            scorerTilter.set(-.5);
          } else if(scorerTilter.getEncoder().getPosition() < Constants.scorerTilterLoadingPosition) {
            scorerTilter.set(.5);
          }
          /* one-time action goes here */
        },
        () -> {
          scorerTilter.set(0);
        }
        ).until(() -> scorerTilter.getEncoder().equals(Constants.scorerTilterLoadingPosition));
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
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.addBooleanProperty("Coral Loaded", () -> isCoralLoaded(), null);
    builder.addBooleanProperty("Over Reef", () -> isCoralOverReef(), null);
  }
}
