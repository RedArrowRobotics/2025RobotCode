package frc.robot.encoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface AngleEncoder {
    public Angle getAngle();
    public AngularVelocity getAngularVelocity();
}
