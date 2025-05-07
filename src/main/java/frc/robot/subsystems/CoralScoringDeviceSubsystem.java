package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.encoder.AngleGenericAbsoluteEncoder;
import edu.wpi.first.units.measure.Angle;

public class CoralScoringDeviceSubsystem extends SubsystemBase {
  SparkMax intakeWheels = new SparkMax(Constants.intakeWheelsMotorID, MotorType.kBrushed);
  public SparkMax scorerTilter = new SparkMax(Constants.coralScorerTilterMotorID, MotorType.kBrushed);
  private DigitalInput coralSensor = new DigitalInput(Constants.coralLimitSwitch);
  private DigitalInput reefSensor = new DigitalInput(Constants.reefPhotoEye);
  public CoralArmPosition target = CoralArmPosition.HOME;
  public CoralArmPosition current = CoralArmPosition.HOME;
  PIDController coralArmPID = new PIDController(0.01, 0.0, 0.0005);
  public double feedForward = 0.0;
  public AngleGenericAbsoluteEncoder angleEncoder;
  boolean manualControl = false;
  double power;
  double encoderOffset = 319.0;

  public CoralScoringDeviceSubsystem() {
    DutyCycleEncoder encoder = new DutyCycleEncoder(3);
    coralArmPID.setTolerance(Degrees.of(2.5).in(Degrees));
    angleEncoder = new AngleGenericAbsoluteEncoder(encoder);
    //coralArmPID.enableContinuousInput(0, 360);
    SmartDashboard.putData("Coral Arm PID", coralArmPID);
  }

  public enum CoralArmPosition {
    HOME(Constants.scorerTilterLoadingPosition),
    SCORE(Constants.scorerTilterScoringPosition),
    NONE(Degrees.of(0.0));

    private Angle encoderPosition;

    private CoralArmPosition(Angle encoderPosition) {
      this.encoderPosition = encoderPosition;
    }

    public Angle getEncoderPosition() {
      return encoderPosition;
    }
  }

  @Override
  public void periodic() {
      if (!manualControl) {
        double adjustedAbsEncoder = angleEncoder.getAngle().in(Degrees)-encoderOffset;
        if(adjustedAbsEncoder < 0) {
          adjustedAbsEncoder += 360;
        }
        power = coralArmPID.calculate(adjustedAbsEncoder, target.getEncoderPosition().in(Degrees)) + feedForward;
        scorerTilter.set(power * 1.0);
      }
  }

  private Command goToPosition(CoralArmPosition targetPosition) {
    return startEnd(
      () -> {
        manualControl = false;
        target = targetPosition;
        current = CoralArmPosition.NONE;
      },
      () -> {
        if(coralArmPID.atSetpoint()) {
          current = targetPosition;
        }
      }
      ).until(() -> coralArmPID.atSetpoint());
  }
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
    ).until(() -> isCoralLoaded()).andThen(grabCoralShort());
  }

  public Command grabCoralShort() {
    return startEnd(
      () -> {
        intakeWheels.set(1);
      },
      () -> {
        intakeWheels.set(0);
      }
    ).withTimeout(Seconds.of(1.0));
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
  public Command scoreCoralPosition() {
      return goToPosition(CoralArmPosition.SCORE);
  }
  
 /**
   * Moves the coral scorer to the correct angle to grab the coral that fell from the chute.
   */
  public Command loadCoralPosition() {
      return goToPosition(CoralArmPosition.HOME);
  }

  public Command coralArmPositive() {
    return startEnd(
      () -> {
        manualControl = true;
        scorerTilter.set(0.5);
      }, () -> {
        scorerTilter.set(0.0);
      }
    );
  }

  public Command coralArmNegative() {
    return startEnd(
      () -> {
        manualControl = true;
        scorerTilter.set(-0.5);
      }, () -> {
        scorerTilter.set(0.0);
      }
    );
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
    return !reefSensor.get();
  }

  public boolean armIsInPosition() {
    return current == CoralArmPosition.SCORE || manualControl == true;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.setSmartDashboardType(getName());
    builder.addBooleanProperty("Coral Loaded", () -> isCoralLoaded(), null);
    builder.addBooleanProperty("Over Reef", () -> isCoralOverReef(), null);
    builder.addStringProperty("Coral Arm Position", () -> current.toString(), null);
    builder.addStringProperty("Coral Arm Target", () -> target.toString(), null);
    builder.addDoubleProperty("Encoder Value", () -> angleEncoder.getAngle().in(Degrees), null);
    builder.addDoubleProperty("Adjusted Encoder Value", () -> angleEncoder.getAngle().in(Degrees) - encoderOffset, null);
    builder.addDoubleProperty("Encoder Target", () -> target.getEncoderPosition().in(Degrees), null);
    builder.addBooleanProperty("Coral Manual Control", () -> manualControl, null);
    builder.addDoubleProperty("Arm Power", () -> power, null);
  }
}
