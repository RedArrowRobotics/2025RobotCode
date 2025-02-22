package frc.robot.encoder;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DistanceEncoder implements AngleEncoder, LinearEncoder {
    AngleEncoder encoder;
    Measure<PerUnit<DistanceUnit,AngleUnit>> travel;

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
        return (Distance) (getAngle().times(travel));
    }
    public LinearVelocity getLinearVelocity() {
        return (LinearVelocity) (getAngularVelocity().times(travel));
    }
}
