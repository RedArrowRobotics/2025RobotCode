package frc.robot.encoder;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface LinearEncoder {
    public Distance getPosition();
    public LinearVelocity getLinearVelocity();
}
