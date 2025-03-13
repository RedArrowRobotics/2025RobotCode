package frc.robot.encoder;

import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.PerUnit;

public class Units {
    public static final PerUnit<DimensionlessUnit,AngleUnit> PulsesPerRevolution = Value.per(Revolution);
}
