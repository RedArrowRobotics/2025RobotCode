import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertFalse;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.encoder.AngleGenericEncoder;

public class Tests {
    @Test
    void passthrough_zero() {
        try(Encoder raw = new Encoder(0, 1)) {
            var encoder = new AngleGenericEncoder(raw);
            assertTrue(encoder.getAngle().isNear(Rotations.zero(),0.01));
        }
    }

    @Test
    void passthrough_rotation() {
        try(Encoder raw = new Encoder(0, 1)) {
            var sim = new EncoderSim(raw);
            sim.setDistance(1.0);
            var encoder = new AngleGenericEncoder(raw);
            assertTrue(encoder.getAngle().isNear(Rotations.one(),0.01));
        }
    }
}
