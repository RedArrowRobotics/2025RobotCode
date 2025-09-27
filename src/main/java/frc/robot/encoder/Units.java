package frc.robot.encoder;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;

public class Units {
    public static final PerUnit<DimensionlessUnit,AngleUnit> PulsesPerRevolution = Value.per(Revolution);
    public static final AngularVelocityUnit RevolutionsPerMinute = Revolutions.per(Minute);
    public static final PerUnit<DistanceUnit,AngleUnit> MetersPerRevolution = Meters.per(Revolution);
    //public static final AngularVelocityUnit BaseAngularVelocityUnit = BaseUnits.AngleUnit.per(BaseUnits.TimeUnit);
    //public static final LinearVelocityUnit BaseLinearVelocityUnit = BaseUnits.DistanceUnit.per(BaseUnits.TimeUnit);
}
