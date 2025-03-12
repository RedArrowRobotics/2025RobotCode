package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Angle;

public class Constants {

    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Named Commands
    public static String ALIGN_REEF_RIGHT = "Align to Reef Right";
    public static String ALIGN_REEF_LEFT = "Align to Reef Left";
    public static String ALIGN_SOURCE = "Align to Source";

    public static String INTAKE_CORAL = "Intake Coral";
    public static String OUTTAKE_CORAL = "Outtake Coral";
    public static String STOP_CORAL_WHEELS = "Stop Coral Wheels";
    public static String CORAL_LOADING_POSITION = "Coral Loading Position";
    public static String CORAL_SCORING_POSITION = "Coral Scoring Position";

    public static String ASCEND_CAGE = "Ascend Cage";
    public static String DESCEND_CAGE = "Descend Cage";

    public static String ELEVATOR_HOME = "Elevator Home";
    public static String SCORE_L2 = "Score L2";
    public static String SCORE_L3 = "Score L3";
    public static String SCORE_L4 = "Score L4";
    public static String EXTEND_ALGAE_REMOVER = "Extend Algae Remover";
    public static String RETRACT_ALGAE_REMOVER = "Retract Algae Remover";
    public static String DEALGAE_ON = "Turn on Dealgae Wheels";
    public static String DEALGAE_OFF = "Turn off Dealgae Wheels";

    // April Tag IDs
    public static List<Integer> blueReefATags = List.of(17,18,19,20,21,22);
    public static List<Integer> redReefATags = List.of(6,7,8,9,10,11);

    public static List<Integer> blueSourceATags = List.of(12, 13);
    public static List<Integer> redSourceATags = List.of(1, 2);
  
    // CANBUS IDs
    public static int intakeWheelsMotorID = 13;
    public static int coralScorerTilterMotorID = 14;
    public static int cageClimberMotorId = 15;
    public static int elevatorMotor1Id = 16;
    public static int elevatorMotor2Id = 17;
    public static int cageGrabberMotorId = 18;
    public static int dealgaeFlipperId = 19;
    public static int dealgaeWheelsId = 20;

    // DIO Ports
    public static int coralSensorChannel = 1;
    public static int reefSensorChannel = 2;
    public static int elevatorMaxSensorChannel = 3;
    public static int elevatorMinSensorChannel = 4;

    // Subsystem Constants
    //   Cage
    public static double cageGrabberReleasePosition = 0.0;
    public static double cageGrabberHoldPosition = 0.5;
    //   Elevator
    public static double elevatorMotorHomePosition = 0;
    public static double elevatorMotorL2Position = 100;
    public static double elevatorMotorL3Position = 200;
    public static double elevatorMotorL4Position = 300;
    public static double dealgaeFlipperExtendedPosition = 1;
    public static double dealgaeFlipperRetractedPosition = 0;

    
    public static Time coralDropTimeout = Seconds.of(3.0);
    public static Angle scorerTilterScoringPosition = Degrees.of(135.0);
    public static Angle scorerTilterLoadingPosition = Degrees.of(0.0);
}
