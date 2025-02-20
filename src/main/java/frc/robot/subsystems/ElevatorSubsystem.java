package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
SparkMax elevatorMotor1 = new SparkMax(Constants.elevatorMotor1Id, MotorType.kBrushless);
SparkMax elevatorMotor2 = new SparkMax(Constants.elevatorMotor2Id, MotorType.kBrushless);
SparkMax dealgaeFlipper = new SparkMax(Constants.dealgaeFlipperId, MotorType.kBrushed);
SparkMax dealgaeWheels = new SparkMax(Constants.dealgaeWheelsId, MotorType.kBrushed);
public ElevatorPositions target = ElevatorPositions.HOME;
public ElevatorPositions current = ElevatorPositions.HOME;
PIDController elevatorPID = new PIDController(0.1, 0, 0);
public double feedForward = 0.0;

public ElevatorSubsystem() {
  //todo: set motor 2 to follow 1
  elevatorPID.setTolerance(10.0);
}

public enum ElevatorPositions {
    HOME(Constants.elevatorMotorHomePosition),
    L2(Constants.elevatorMotorL2Position),
    L3(Constants.elevatorMotorL3Position),
    L4(Constants.elevatorMotorL4Position),
    NONE(0.0);

    private double encoderPosition;

    private ElevatorPositions(double encoderPosition) {
      this.encoderPosition = encoderPosition;
    }

    public double getEncoderPosition() {
      return encoderPosition;
    }
}

@Override
public void periodic() {
  double power;
  power = elevatorPID.calculate(elevatorMotor1.getEncoder().getPosition(), target.getEncoderPosition()) + feedForward;
  elevatorMotor1.setVoltage(power);
}

  /**
   * Moves the elevator to the home position which is the lowest position.
   */
  public Command elevatorHome() {
    return goToPosition(ElevatorPositions.HOME);
  }

  /**
   * Moves the elevator to the L2 position.
   */
  public Command elevatorL2() {
    return goToPosition(ElevatorPositions.L2);
  }
  
  /**
   * Moves the elevator to the L3 position.
   */
  public Command elevatorL3() {
    return goToPosition(ElevatorPositions.L3);
  }
  
  /**
   * Moves the elevator to the L4 position.
   */
  public Command elevatorL4() {
    return goToPosition(ElevatorPositions.L4);
  }

  private Command goToPosition(ElevatorPositions targetPosition) {
    return startEnd(
      () -> {
        target = targetPosition;
        current = ElevatorPositions.NONE;
      },
      () -> {
        current = targetPosition;
      }
      ).until(() -> elevatorPID.atSetpoint());
  }
  
  /**
   * Moves the dealgaer into the dealgae position.
   */
  public Command dealgaeExtend() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> dealgaeFlipper.getEncoder().equals(Constants.dealgaeFlipperExtendedPosition));
        //getEncoder or getPosition?
  }
  
  /**
   * Moves the dealgaer into the stored position.
   */
  public Command dealgaeRetract() {
    return startEnd(
        () -> {
          elevatorMotor1.set(.5);
        },
        () -> {
          elevatorMotor1.set(0);
        }
        ).until(() -> dealgaeFlipper.getEncoder().equals(Constants.dealgaeFlipperRetractedPosition));
        //getEncoder or getPosition?
  }
  
  /**
   * Activates the dealgaer.
   */
  public Command dealgaeStartSpin() {
    return runOnce(
        () -> {
          dealgaeWheels.set(1);
        });
  }
  
  /**
   * Deactivates the dealgaer.
   */
  public Command dealgaeStopSpin() {
    return runOnce(
        () -> {
          dealgaeWheels.set(0);
        });
  }
}
