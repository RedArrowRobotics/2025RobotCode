package frc.robot.encoder;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;

public class DistanceEncoder implements AngleEncoder, LinearEncoder {
    AngleEncoder encoder;
    Measure<PerUnit<DistanceUnit,AngleUnit>> travel;
    MutDistance storage_distance = Meters.mutable(0);
    MutLinearVelocity storage_rate = MetersPerSecond.mutable(0);

    /**
     * Creates a new distance-based encoder.
     * @param encoder - angle-based encoder to wrap
     * @param travel - distance the attached mechanism travels per unit of angle (for example, the circumerence of a wheel)
     */
    @SuppressWarnings("unchecked")
    public <T extends PerUnit<DistanceUnit,AngleUnit>> DistanceEncoder(AngleEncoder encoder, Measure<T> travel) {
        this.encoder = encoder;
        this.travel = (Measure<PerUnit<DistanceUnit,AngleUnit>>) travel;
    }

    public Angle getAngle() {
        return encoder.getAngle();
    }
    public AngularVelocity getAngularVelocity() {
        return encoder.getAngularVelocity();
    }
    public Distance getPosition() {
        storage_distance.mut_replace(getAngle().in(Revolutions)*travel.in(Units.MetersPerRevolution), Meters);
        return storage_distance;
    }
    public LinearVelocity getLinearVelocity() {
        storage_rate.mut_replace(getAngularVelocity().in(RevolutionsPerSecond)*travel.in(Units.MetersPerRevolution), MetersPerSecond);
        return storage_rate;
    }
}
