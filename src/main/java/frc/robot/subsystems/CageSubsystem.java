package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.encoder.AngleGenericAbsoluteEncoder;
import frc.robot.encoder.AngleSparkRelativeEncoder;
import frc.robot.subsystems.CoralScoringDeviceSubsystem.CoralArmPosition;

public class CageSubsystem extends SubsystemBase {
  SparkFlex cageClimber = new SparkFlex(Constants.cageClimberMotorId, MotorType.kBrushless);
  private DigitalInput cageSensor = new DigitalInput(Constants.cageSensorChannel);
  SparkMax cageGrabber = new SparkMax(Constants.cageGrabberMotorId, MotorType.kBrushless);
  PIDController cageGrabberPID = new PIDController(0.03, 0.0, 0.0);
  public double feedForward = 0.0;
 // public AngleSparkRelativeEncoder angleEncoder;
  public CageGrabberPosition target = CageGrabberPosition.CLOSED;
  public CageGrabberPosition current = CageGrabberPosition.CLOSED;

  public CageSubsystem() {
    cageGrabberPID.setTolerance(0.1);
    //angleEncoder = new AngleSparkRelativeEncoder(cageGrabber.getEncoder(), Value.of(42).div(Revolutions.one()), .05);
    //cageGrabberPID.enableContinuousInput(0, 360);
  }

  public enum CageGrabberPosition {
    OPEN(Constants.cageGrabberOpenPosition),
    CLOSED(Constants.cageGrabberClosedPosition),
    NONE(0.0);

    private double encoderPosition;

    private CageGrabberPosition(double encoderPosition) {
      this.encoderPosition = encoderPosition;
    }

    public double getEncoderPosition() {
      return encoderPosition;
    }
  }

  @Override
  public void periodic() {
    double power;
    power = cageGrabberPID.calculate(cageGrabber.getEncoder().getPosition(), target.getEncoderPosition()) * -1.0
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
          cageClimber.set(-0.1);
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

  public boolean isCageAtLimit() {
    return cageSensor.get();
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.addBooleanProperty("Cage Closed", () -> cageIsClosed(), null);
    builder.addStringProperty("Arm Position", () -> current.toString(), null);
    builder.addStringProperty("Arm Target", () -> target.name(), null);
    builder.addDoubleProperty("Encoder Value", () -> cageGrabber.getEncoder().getPosition(), null);
    builder.addDoubleProperty("Encoder Target", () -> target.getEncoderPosition(), null);
  }
}
