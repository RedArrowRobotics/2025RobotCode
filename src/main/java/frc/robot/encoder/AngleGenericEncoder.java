package frc.robot.encoder;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Encoder;

public class AngleGenericEncoder implements AngleEncoder {
    Encoder encoder;
    Measure<PerUnit<DimensionlessUnit,AngleUnit>> ppm;
    double gearRatio;
    
    /**
     * Creates a new angle-based encoder from a generic external encoder.
     * @param encoder - the relative encoder to wrap.
     */
    public AngleGenericEncoder(Encoder encoder) {
        this(encoder,Value.one().div(Revolutions.one()),1.0);
    }

    /**
     * Creates a new angle-based encoder from a generic external encoder.
     * @param encoder - the relative encoder to wrap.
     * @param gearRatio - an additional multiplier on the reported angle
     */
    public AngleGenericEncoder(Encoder encoder, double gearRatio) {
        this(encoder,Value.one().div(Revolutions.one()),gearRatio);
    }

    /**
     * Creates a new angle-based encoder from a generic external encoder.
     * @param encoder - the relative encoder to wrap.
     * @param ppm - the ratio of encoder pulses to motor revolutions
     */
    public <T extends PerUnit<DimensionlessUnit,AngleUnit>> AngleGenericEncoder(Encoder encoder, Measure<T> ppm) {
        this(encoder,ppm,1.0);
    }

    /**
     * Creates a new angle-based encoder from a generic external encoder.
     * @param encoder - the relative encoder to wrap.
     * @param ppm - the ratio of encoder pulses to motor revolutions
     * @param gearRatio - an additional multiplier on the reported angle
     */
    @SuppressWarnings("unchecked")
    public <T extends PerUnit<DimensionlessUnit,AngleUnit>> AngleGenericEncoder(Encoder encoder, Measure<T> ppm, double gearRatio) {
        this.encoder = encoder;
        this.ppm = (Measure<PerUnit<DimensionlessUnit,AngleUnit>>) ppm;
        this.gearRatio = gearRatio;
    }

    public Angle getAngle() {
        var position = Value.of(encoder.getDistance());
        var revolutions = BaseUnits.AngleUnit.of(position.div(ppm).baseUnitMagnitude());
        var geared = revolutions.times(gearRatio);
        return geared;
    }

    public AngularVelocity getAngularVelocity() {
        var position = Value.of(encoder.getRate()).div(Seconds.one());
        var revolutions = BaseUnits.AngleUnit.per(BaseUnits.TimeUnit).of(position.div(ppm).baseUnitMagnitude());
        var geared = revolutions.times(gearRatio);
        return geared;
    }
}
