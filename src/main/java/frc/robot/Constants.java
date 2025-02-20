package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Time;

public class Constants {

    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static List<Integer> blueReefATags = List.of(17,18,19,20,21,22);
    public static List<Integer> redReefATags = List.of(6,7,8,9,10,11);

    public static int elevatorMotor1Id = 16;
    public static int elevatorMotor2Id = 17;
    public static int dealgaeFlipperId = 19;
    public static int dealgaeWheelsId = 20;
    public static int cageGrabberMotorId = 18;

    public static int cageClimberMotorId = 15;

    public static double cageGrabberReleasePosition = 0.0;
    public static double cageGrabberHoldPosition = 0.5;
    public static double elevatorMotorHomePosition = 0;
    public static double elevatorMotorL2Position = 2;
    public static double elevatorMotorL3Position = 3;
    public static double elevatorMotorL4Position = 4;
    public static double dealgaeFlipperExtendedPosition = 1;
    public static double dealgaeFlipperRetractedPosition = 0;

    public static int intakeWheelsMotorID = 13;
    public static int coralScorerTilterMotorID = 14;

    public static int coralSensorChannel = 1;
    public static int reefSensorChannel = 2;

    public static Time coralDropTimeout = Seconds.of(3.0);
    public static double scorerTilterScoringPositionL2L3 = 10.0;
    public static double scorerTilterScoringPositionL4 = 20.0;
    public static double scorerTilterLoadingPosition = 0.0;
}
