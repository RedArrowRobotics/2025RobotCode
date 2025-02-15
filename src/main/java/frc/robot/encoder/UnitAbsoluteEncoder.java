package frc.robot.encoder;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;

public class UnitAbsoluteEncoder implements AngleEncoder {
    SparkAbsoluteEncoder encoder;
    Dimensionless gearRatio;

    public UnitAbsoluteEncoder(SparkMax controller) {
        this(controller,Value.one());
    }
    public UnitAbsoluteEncoder(SparkMax controller, Dimensionless gearRatio) {
        encoder = controller.getAbsoluteEncoder();
        this.gearRatio = gearRatio;
    }

    public Angle getPosition() {
        return Revolutions.of(encoder.getPosition()).div(gearRatio);
    }

    public AngularVelocity getVelocity() {
        return Revolutions.per(Minute).of(encoder.getVelocity()).div(gearRatio);
    }
}
