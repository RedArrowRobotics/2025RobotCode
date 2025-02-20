package frc.robot.encoder;

import static edu.wpi.first.units.Units.Minutes;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class AngleAbsoluteEncoder implements AngleEncoder {
    AbsoluteEncoder encoder;
    Measure<PerUnit<DimensionlessUnit,AngleUnit>> ppm;
    double gearRatio;
    
    /**
     * Creates a new angle-based encoder from an absolute encoder.
     * @param encoder - the relative encoder to wrap.
     */
    public AngleAbsoluteEncoder(AbsoluteEncoder encoder) {
        this(encoder,Value.one().div(Revolutions.one()),1.0);
    }
    
    /**
     * Creates a new angle-based encoder from an absolute encoder.
     * @param encoder - the relative encoder to wrap.
     * @param gearRatio - an additional multiplier on the reported angle
     */
    public AngleAbsoluteEncoder(AbsoluteEncoder encoder, double gearRatio) {
        this(encoder,Value.one().div(Revolutions.one()),gearRatio);
    }
    
    /**
     * Creates a new angle-based encoder from an absolute encoder.
     * @param encoder - the relative encoder to wrap.
     * @param ppm - the ratio of encoder pulses to motor revolutions
     */
    public AngleAbsoluteEncoder(AbsoluteEncoder encoder, Measure<? extends PerUnit<DimensionlessUnit,AngleUnit>> ppm) {
        this(encoder,ppm,1.0);
    }
    
    /**
     * Creates a new angle-based encoder from an absolute encoder.
     * @param encoder - the relative encoder to wrap.
     * @param ppm - the ratio of encoder pulses to motor revolutions
     * @param gearRatio - an additional multiplier on the reported angle
     */
    @SuppressWarnings("unchecked")
    public AngleAbsoluteEncoder(AbsoluteEncoder encoder, Measure<? extends PerUnit<DimensionlessUnit,AngleUnit>> ppm, double gearRatio) {
        this.encoder = encoder;
        this.ppm = (Measure<PerUnit<DimensionlessUnit,AngleUnit>>) ppm;
        this.gearRatio = gearRatio;
    }

    public Angle getAngle() {
        var position = Value.of(encoder.getPosition());
        var revolutions = BaseUnits.AngleUnit.of(position.div(ppm).baseUnitMagnitude());
        var geared = revolutions.times(gearRatio);
        return geared;
    }

    public AngularVelocity getAngularVelocity() {
        var position = Value.of(encoder.getVelocity()).div(Minutes.one());
        var revolutions = BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(position.div(ppm).baseUnitMagnitude());
        var geared = revolutions.times(gearRatio);
        return geared;
    }
}
