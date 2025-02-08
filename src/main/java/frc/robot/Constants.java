package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {

    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static List<Integer> blueReefATags = List.of(17,18,19,20,21,22);
    public static List<Integer> redReefATags = List.of(6,7,8,9,10,11);

    public static int intakeWheelsID = 13;
    public static int coralScorerTilterID = 14;

    public static int coralSensorChannel = 1;
    public static int reefSensorChannel = 2;

    public static double coralDropTimeoutInSeconds = 3;
}
