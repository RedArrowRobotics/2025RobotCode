package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Optional;

import com.revrobotics.spark.SparkBase;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevatorMotor1 = new SparkMax(Constants.elevatorMotor1Id, MotorType.kBrushless);
    SparkMax elevatorMotor2 = new SparkMax(Constants.elevatorMotor2Id, MotorType.kBrushless);
    SparkMax dealgaeFlipper = new SparkMax(Constants.dealgaeFlipperId, MotorType.kBrushed);
    SparkMax dealgaeWheels = new SparkMax(Constants.dealgaeWheelsId, MotorType.kBrushed);
    public ElevatorPositions target = ElevatorPositions.HOME;
    public ElevatorPositions current = ElevatorPositions.HOME;
    //PIDController elevatorPID = new PIDController(0.025, 0, 0.001);
    //public double feedForward = 0.03;
    SparkMaxConfig config = new SparkMaxConfig();
    Optional<Double> manualControl = Optional.empty();
    //  Motion Profile
    ProfiledPIDController elevatorProfile = new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Constants.maxVelocity, Constants.maxAcceleration));
    ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(1.1, 1.2, 1.3);

    public ElevatorSubsystem() {
        // todo: set motor 2 to follow 1
        elevatorProfile.setTolerance(Constants.elevatorTolerance);
        config.follow(elevatorMotor1);
        elevatorMotor2.configure(config, SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters);
        SmartDashboard.putData("Elevator PID", elevatorProfile);
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
        manualControl.ifPresentOrElse((power) -> {
            elevatorMotor1.set(power + elevatorFeedforward.calculate(0));
        }, () -> {
            elevatorMotor1.set(elevatorProfile.calculate(elevatorMotor1.getEncoder().getPosition(), target.getEncoderPosition()) + elevatorFeedforward.calculate(elevatorProfile.getSetpoint().velocity));
        });
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
                manualControl = Optional.empty();
                target = targetPosition;
                //elevatorProfile.calculate(elevatorMotor1.getEncoder().getPosition(), target.getEncoderPosition());
                elevatorProfile.setGoal(target.getEncoderPosition());
                current = ElevatorPositions.NONE;
            },
            () -> {
                if (elevatorProfile.atGoal()) {
                    current = targetPosition;
                }
            }).until(() -> elevatorProfile.atGoal());
    }

    public Boolean isElevatorAtL2() {
        return elevatorMotor1.getEncoder().getPosition() >= Constants.elevatorMotorL2Position
            - Constants.elevatorTolerance;
    }

    public Command raiseElevator() {
        return startEnd(
            () -> {
                manualControl = Optional.of(0.1);
            }, () -> {
                manualControl = Optional.empty();
            }
        );
    }

    public Command lowerElevator() {
        return startEnd(
            () -> {
                manualControl = Optional.of(-0.1);
            }, () -> {
                manualControl = Optional.empty();
            }
        );
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
            }).until(() -> dealgaeFlipper.getEncoder().equals(Constants.dealgaeFlipperExtendedPosition));
        // getEncoder or getPosition?
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
            }).until(() -> dealgaeFlipper.getEncoder().equals(Constants.dealgaeFlipperRetractedPosition));
        // getEncoder or getPosition?
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

    public boolean elevatorIsInPosition() {
        return current == ElevatorPositions.L2 || current == ElevatorPositions.L3 || current == ElevatorPositions.L4 || manualControl.isPresent();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType(getName());
        builder.addDoubleProperty("Elevator Motor Position", () -> elevatorMotor1.getEncoder().getPosition(), null);
        builder.addStringProperty("Elevator Position", () -> current.toString(), null);
        builder.addDoubleProperty("Elevator Manual Control", () -> manualControl.orElse(null), null);
        builder.addStringProperty("Alliance Color", () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).name(), null);
        builder.addBooleanProperty("Elevator at L2", () -> isElevatorAtL2(), null);
    }
}
