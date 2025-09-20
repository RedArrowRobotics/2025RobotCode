package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdWrapper {
    private final String name;
    private final SysIdRoutine sysIdRoutine;
    private MutVoltage voltage = Volts.mutable(0);
    private MutAngle angularPosition = Rotations.mutable(0);
    private MutAngularVelocity angularVelocity = RotationsPerSecond.mutable(0);
    private MutDistance linearPosition = Meters.mutable(0);
    private MutLinearVelocity linearVelocity = MetersPerSecond.mutable(0);

    public SysIdWrapper(Properties properties) {
        name = properties.name;
        sysIdRoutine = new SysIdRoutine(
                properties.config,
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (voltage) -> {for (MotorController controller : properties.motorControllers) {controller.motorController.setVoltage(controller.reverse ? voltage.unaryMinus() : voltage);}},
                        // Tell SysId how to record a frame of data for each
                        // motor on the mechanism being characterized.
                        log -> {
                            // Record a frame for the shooter motor.
                            var motorLog = log.motor(properties.name)
                                .voltage(voltage.mut_replace(
                                    properties.motorControllers.get(0).motorController.get() * RobotController.getBatteryVoltage(), Volts));
                            properties.metersPerRotation.ifPresentOrElse(metersPerRotation -> {
                                motorLog.linearPosition(linearPosition.mut_replace(
                                    properties.motorControllers.get(0).motorController.getEncoder().getPosition() * metersPerRotation, Meters))
                                .linearVelocity(linearVelocity.mut_replace(
                                    properties.motorControllers.get(0).motorController.getEncoder().getVelocity() * metersPerRotation, Meters.per(Minute)));
                            }, () -> {
                                motorLog.angularPosition(angularPosition.mut_replace(
                                    properties.motorControllers.get(0).motorController.getEncoder().getPosition(), Rotations))
                                .angularVelocity(angularVelocity.mut_replace(
                                    properties.motorControllers.get(0).motorController.getEncoder().getVelocity(), RPM));
                            });
                        },
                        properties.subsystem));
    }

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    private Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public void sendCommandsToDashboard() {
        SmartDashboard.putData("sysid-"+this.name+ "-quasistatic-forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("sysid-"+this.name+ "-quasistatic-reverse", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("sysid-"+this.name+ "-dynamic-forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("sysid-"+this.name+ "-dynamic-reverse", sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public static class Properties {
        private String name;
        private SysIdRoutine.Config config;
        private List<MotorController> motorControllers;
        private Subsystem subsystem;
        private Optional<Double> metersPerRotation;
    
        public Properties(String name, SysIdRoutine.Config config, List<MotorController> motorControllers, Subsystem subsystem) {
            this.name = name;
            this.config = config;
            this.motorControllers = motorControllers;
            this.subsystem = subsystem;
            this.metersPerRotation = Optional.empty();
        }
        public Properties(String name, SysIdRoutine.Config config, List<MotorController> motorControllers, Subsystem subsystem, double metersPerRotation) {
            this.name = name;
            this.config = config;
            this.motorControllers = motorControllers;
            this.subsystem = subsystem;
            this.metersPerRotation = Optional.of(metersPerRotation);
        }
    }
    public static record MotorController(SparkMax motorController, boolean reverse) {
    }
}
