package frc.robot.encoder;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class DistanceEncoder {
    AngleEncoder encoder;
    Measure<PerUnit<DistanceUnit,AngleUnit>> travel;

    public DistanceEncoder(AngleEncoder encoder, Measure<PerUnit<DistanceUnit,AngleUnit>> travel) {
        this.encoder = encoder;
    }

    public Distance getPosition() {
        return (Distance) (encoder.getPosition().times(travel));
    }
    public LinearVelocity getVelocity() {
        return (LinearVelocity) (encoder.getVelocity().times(travel));
    }
}
