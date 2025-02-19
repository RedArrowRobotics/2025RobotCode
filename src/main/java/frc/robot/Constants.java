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

    public static int intakeWheelsMotorID = 13;
    public static int coralScorerTilterMotorID = 14;

    public static int coralSensorChannel = 1;
    public static int reefSensorChannel = 2;

    public static Time coralDropTimeout = Seconds.of(3.0);
    public static double scorerTilterScoringPositionL2L3 = 10.0;
    public static double scorerTilterScoringPositionL4 = 20.0;
    public static double scorerTilterLoadingPosition = 0.0;
}
