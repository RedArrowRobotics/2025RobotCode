package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.encoder.AngleEncoder;
import frc.robot.encoder.LinearEncoder;

public class SysIdWrapper {
    private final String name;
    private final SysIdRoutine sysIdRoutine;

    public SysIdWrapper(Properties properties) {
        name = properties.name;
        sysIdRoutine = new SysIdRoutine(
                properties.config,
                new SysIdRoutine.Mechanism(
                        // Tell SysId how to plumb the driving voltage to the motors.
                        (voltage) -> properties.system.runMotors(voltage),
                        // Tell SysId how to record a frame of data for each
                        // motor on the mechanism being characterized.
                        log -> properties.system.log(log, properties.name),
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
        SmartDashboard.putData("System Identification/"+this.name+"/Quasistatic/Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("System Identification/"+this.name+"/Quasistatic/Reverse", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData("System Identification/"+this.name+"/Dynamic/Forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
        SmartDashboard.putData("System Identification/"+this.name+"/Dynamic/Reverse", sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public static class Properties {
        private String name;
        private SysIdRoutine.Config config;
        private System system;
        private Subsystem subsystem;
    
        public Properties(String name, SysIdRoutine.Config config, AngularMotorList motorControllers, Subsystem subsystem) {
            this.name = name;
            this.config = config;
            this.system = new AngularSystem(motorControllers.get());
            this.subsystem = subsystem;
        }
        public Properties(String name, SysIdRoutine.Config config, LinearMotorList motorControllers, Subsystem subsystem) {
            this.name = name;
            this.config = config;
            this.system = new LinearSystem(motorControllers.get());
            this.subsystem = subsystem;
        }
    }

    private interface System {
        public void runMotors(Voltage voltage);
        public void log(SysIdRoutineLog log, String name);
    }
    public static class AngularSystem implements System {
        private List<AngularMotorController> motorControllers;
        private MutVoltage voltage = Volts.mutable(0);
        private MutAngle angularPosition = Rotations.mutable(0);
        private MutAngularVelocity angularVelocity = RotationsPerSecond.mutable(0);

        public AngularSystem(List<AngularMotorController> motorControllers) {
            this.motorControllers = motorControllers;
        }
        public void runMotors(Voltage voltage) {
            for (AngularMotorController controller : motorControllers) {
                controller.motorController.setVoltage(voltage);
            }
        }
        public void log(SysIdRoutineLog log, String name) {
            for (int i=0; i<motorControllers.size(); i++) {
                final AngularMotorController motor = motorControllers.get(i);
                var motorLog = log.motor(name+motor.name.map(subname -> "-"+subname).orElse(""));
                motorLog.voltage(voltage.mut_replace(
                    motor.motorController.get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(angularPosition.mut_replace(motor.encoder.getAngle()))
                .angularVelocity(angularVelocity.mut_replace(motor.encoder.getAngularVelocity()));
            }
        }
    }
    public static class LinearSystem implements System {
        private List<LinearMotorController> motorControllers;
        private MutVoltage voltage = Volts.mutable(0);
        private MutDistance linearPosition = Meters.mutable(0);
        private MutLinearVelocity linearVelocity = MetersPerSecond.mutable(0);
        
        public LinearSystem(List<LinearMotorController> motorControllers) {
            this.motorControllers = motorControllers;
        }
        public void runMotors(Voltage voltage) {
            for (LinearMotorController controller : motorControllers) {
                controller.motorController.setVoltage(voltage);
            }
        }
        public void log(SysIdRoutineLog log, String name) {
            for (int i=0; i<motorControllers.size(); i++) {
                final LinearMotorController motor = motorControllers.get(i);
                var motorLog = log.motor(name+motor.name.map(subname -> "-"+subname).orElse(""));
                motorLog.voltage(voltage.mut_replace(
                    motor.motorController.get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(linearPosition.mut_replace(motor.encoder.getPosition()))
                .linearVelocity(linearVelocity.mut_replace(motor.encoder.getLinearVelocity()));
            }
        }
    }

    public interface AngularMotorList extends Supplier<List<AngularMotorController>> {}
    public interface LinearMotorList extends Supplier<List<LinearMotorController>> {}
    public static record AngularMotorController(SparkMax motorController, AngleEncoder encoder, Optional<String> name) {}
    public static record LinearMotorController(SparkMax motorController, LinearEncoder encoder, Optional<String> name) {}
}
