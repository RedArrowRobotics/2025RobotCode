package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CageSubsystem extends SubsystemBase {
  SparkFlex cageClimber = new SparkFlex(Constants.cageClimberMotorId, MotorType.kBrushless);

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
