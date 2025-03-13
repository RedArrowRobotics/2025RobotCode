package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.encoder.AngleGenericAbsoluteEncoder;
import frc.robot.encoder.AngleSparkRelativeEncoder;
import frc.robot.subsystems.CoralScoringDeviceSubsystem.CoralArmPosition;

public class CageSubsystem extends SubsystemBase {
  SparkFlex cageClimber = new SparkFlex(Constants.cageClimberMotorId, MotorType.kBrushless);
  SparkMax cageGrabber = new SparkMax(Constants.cageGrabberMotorId, MotorType.kBrushless);
  PIDController cageGrabberPID = new PIDController(0.03, 0.0, 0.0);
  public double feedForward = 0.0;
  public AngleSparkRelativeEncoder angleEncoder;
  public CageGrabberPosition target = CageGrabberPosition.OPEN;
  public CageGrabberPosition current = CageGrabberPosition.OPEN;

  public CageSubsystem() {
    cageGrabberPID.setTolerance(Degrees.of(1.0).in(Degrees));
    angleEncoder = new AngleSparkRelativeEncoder(cageGrabber.getEncoder(),.2);
    //cageGrabberPID.enableContinuousInput(0, 360);
  }

  public enum CageGrabberPosition {
    OPEN(Constants.scorerTilterLoadingPosition),
    CLOSED(Constants.scorerTilterScoringPosition),
    NONE(Degrees.of(0.0));

    private Angle encoderPosition;

    private CageGrabberPosition(Angle encoderPosition) {
      this.encoderPosition = encoderPosition;
    }

    public Angle getEncoderPosition() {
      return encoderPosition;
    }
  }

  @Override
  public void periodic() {
    double power;
    power = cageGrabberPID.calculate(angleEncoder.getAngle().in(Degrees), target.getEncoderPosition().in(Degrees))
        + feedForward;
    cageGrabber.set(power * -1.0);
  }

  /**
   * Pushes down on the cage to raise the robot.
   */
  public Command ascend() {
    return startEnd(
        () -> {
          cageClimber.set(1.0);
        },
        () -> {
          cageClimber.set(0);
        });
  }

  /**
   * Retracts the climber to lower the robot.
   */
  public Command descend() {
    return startEnd(
        () -> {
          cageClimber.set(-0.25);
        },
        () -> {
          cageClimber.set(0);
        });
  }

  private Command goToPosition(CageGrabberPosition targetPosition) {
    return startEnd(
        () -> {
          target = targetPosition;
          current = CageGrabberPosition.NONE;
        },
        () -> {
          if (cageGrabberPID.atSetpoint()) {
            current = targetPosition;
          }
        }).until(() -> cageGrabberPID.atSetpoint());
  }

  /**
  *
  */
  public Command cageGrabberOpenPosition() {
    return goToPosition(CageGrabberPosition.OPEN);
  }

  /**
   *
   */
  public Command cageGrabberClosedPosition() {
    return goToPosition(CageGrabberPosition.CLOSED);
  }

  /**
   * Moves the coral scorer to the correct angle to drop it on the reef.
   */
  public boolean cageIsOpen() {
    return current == CageGrabberPosition.OPEN;
  }

  /**
   * Moves the coral scorer to the correct angle to grab the coral that fell from
   * the chute.
   */
  public boolean cageIsClosed() {
    return current == CageGrabberPosition.CLOSED;
  }

  /**
   * Checks if the cage is in the holding position.
   */
  public boolean isCageHeld() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
