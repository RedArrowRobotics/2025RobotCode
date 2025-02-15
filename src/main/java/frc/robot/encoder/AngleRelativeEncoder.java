package frc.robot.encoder;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;

public class AngleRelativeEncoder implements AngleEncoder {
    RelativeEncoder encoder;
    Dimensionless gearRatio;

    public AngleRelativeEncoder(RelativeEncoder encoder) {
        this(encoder,Value.one());
    }
    public AngleRelativeEncoder(RelativeEncoder encoder, Dimensionless gearRatio) {
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
