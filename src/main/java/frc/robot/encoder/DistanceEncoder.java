package frc.robot.encoder;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DistanceEncoder implements AngleEncoder, LinearEncoder {
    AngleEncoder encoder;
    Measure<PerUnit<DistanceUnit,AngleUnit>> travel;

    public DistanceEncoder(AbsoluteEncoder encoder, Measure<PerUnit<DistanceUnit,AngleUnit>> travel) {
        this(encoder, Value.one(), travel);
    }
    public DistanceEncoder(AbsoluteEncoder encoder, Dimensionless gearRatio, Measure<PerUnit<DistanceUnit,AngleUnit>> travel) {
        this(new AngleAbsoluteEncoder(null, gearRatio), travel);
    }
    public DistanceEncoder(RelativeEncoder encoder, Measure<PerUnit<DistanceUnit,AngleUnit>> travel) {
        this(encoder, Value.one(), travel);
    }
    public DistanceEncoder(RelativeEncoder encoder, Dimensionless gearRatio, Measure<PerUnit<DistanceUnit,AngleUnit>> travel) {
        this(new AngleRelativeEncoder(null, gearRatio), travel);
    }
    public DistanceEncoder(AngleEncoder encoder, Measure<PerUnit<DistanceUnit,AngleUnit>> travel) {
        this.encoder = encoder;
        this.travel = travel;
    }

    public Angle getAngle() {
        return encoder.getAngle();
    }
    public AngularVelocity getAngularVelocity() {
        return encoder.getAngularVelocity();
    }
    public Distance getPosition() {
        return (Distance) (getAngle().times(travel));
    }
    public LinearVelocity getLinearVelocity() {
        return (LinearVelocity) (getAngularVelocity().times(travel));
    }
}
