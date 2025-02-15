package frc.robot.encoder;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;

public class AngleAbsoluteEncoder implements AngleEncoder {
    AbsoluteEncoder encoder;
    Dimensionless gearRatio;

    public AngleAbsoluteEncoder(AbsoluteEncoder encoder) {
        this(encoder,Value.one());
    }
    public AngleAbsoluteEncoder(AbsoluteEncoder encoder, Dimensionless gearRatio) {
        this.encoder = encoder;
        this.gearRatio = gearRatio;
    }

    public Angle getAngle() {
        return Revolutions.of(encoder.getPosition()).times(gearRatio);
    }

    public AngularVelocity getAngularVelocity() {
        return Revolutions.per(Minute).of(encoder.getVelocity()).times(gearRatio);
    }
}
