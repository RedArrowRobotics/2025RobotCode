import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.encoder.AngleGenericRelativeEncoder;
import frc.robot.encoder.DistanceEncoder;

public class EncoderTests {
    /**
     * Test that an encoder that hasn't rotated returns an angle of 0°.
     */
    @Test
    void passthrough_rotation_null() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setDistance(0.0);
            var encoder = new AngleGenericRelativeEncoder(raw);
            assertEquals(Rotations.zero().baseUnitMagnitude(),encoder.getAngle().baseUnitMagnitude(), 0.01);
        }
    }

    /**
     * Test that an encoder that has rotated a full rotation returns an angle of 360°.
     */
    @Test
    void passthrough_rotation() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setDistance(1.0);
            var encoder = new AngleGenericRelativeEncoder(raw);
            assertEquals(Rotations.one().baseUnitMagnitude(),encoder.getAngle().baseUnitMagnitude(), 0.01);
        }
    }

    /**
     * Test that an encoder geared 1:2 rotates half the distance.
     */
    @Test
    void geared_rotation() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setDistance(1.0);
            var encoder = new AngleGenericRelativeEncoder(raw, 0.5);
            assertEquals(Degrees.of(180).baseUnitMagnitude(),encoder.getAngle().baseUnitMagnitude(), 0.01);
        }
    }

    /**
     * Test that an encoder with 2 PPM rotates at half speed.
     */
    @Test
    void ppm_rotation() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setDistance(1.0);
            var encoder = new AngleGenericRelativeEncoder(raw, Value.per(Rotations).of(2));
            assertEquals(Degrees.of(180).baseUnitMagnitude(),encoder.getAngle().baseUnitMagnitude(), 0.01);
        }
    }

    /**
     * Test that PPM and gear ratio have combined effect.
     */
    @Test
    void ppm_geared_rotation() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setDistance(1.0);
            var encoder = new AngleGenericRelativeEncoder(raw, Value.per(Rotations).of(2), 0.5);
            assertEquals(Degrees.of(90).baseUnitMagnitude(),encoder.getAngle().baseUnitMagnitude(), 0.01);
        }
    }

    /**
     * Test that a linear encoder reports the correct distance.
     */
    @Test
    void linear() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setDistance(1.0);
            var encoder = new DistanceEncoder(
                new AngleGenericRelativeEncoder(raw, Value.per(Rotations).of(2), 0.5), 
                Centimeters.per(Rotations).of(8)
            );
            assertEquals(Centimeters.of(2).baseUnitMagnitude(),encoder.getPosition().baseUnitMagnitude(), 0.01);
        }
    }

    /**
     * Test that an encoder reports the correct rate.
     */
    @Test
    void rate() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setRate(1.0);
            var encoder = new DistanceEncoder(
                new AngleGenericRelativeEncoder(raw, Value.per(Rotations).of(2), 0.5), 
                Centimeters.per(Rotations).of(8)
            );
            assertEquals(Centimeters.per(Seconds).of(2).baseUnitMagnitude(),encoder.getPosition().baseUnitMagnitude(), 0.01);
        }
    }
}
