package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {

    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static List<Integer> blueReefATags = List.of(17,18,19,20,21,22);
    public static List<Integer> redReefATags = List.of(6,7,8,9,10,11);

    public static int elevatorMotorId = 16; //16 and 17 for elevator, need two motors
    public static int deAlgaeFlipperId = 19;
    public static int deAlgaeWheelsId = 20;
    public static int cageClimberMotorId = 15;

    public static double cageGrabberReleasePosition = 0.0;
    public static double cageGrabberHoldPosition = 0.5;

    public static int intakeWheelsMotorID = 13;
    public static int coralScorerTilterMotorID = 14;

    public static int coralSensorChannel = 1;
    public static int reefSensorChannel = 2;

    public static double coralDropTimeoutInSeconds = 3;
}
