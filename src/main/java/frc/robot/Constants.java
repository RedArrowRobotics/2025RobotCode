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
    public final static String ALIGN_REEF_RIGHT = "Align to Reef Right";
    public final static String ALIGN_REEF_LEFT = "Align to Reef Left";
    public final static String ALIGN_SOURCE = "Align to Source";

    public final static String INTAKE_CORAL = "Intake Coral";
    public final static String OUTTAKE_CORAL = "Outtake Coral";
    public final static String STOP_CORAL_WHEELS = "Stop Coral Wheels";
    public final static String CORAL_LOADING_POSITION = "Coral Loading Position";
    public final static String CORAL_SCORING_POSITION = "Coral Scoring Position";
    public final static String MANUAL_CORAL_POSITIVE = "Manual Coral Load";
    public final static String MANUAL_CORAL_NEGATIVE = "Manual Coral Score";

    public final static String ASCEND_CAGE = "Ascend Cage";
    public final static String DESCEND_CAGE = "Descend Cage";

    public final static String ELEVATOR_HOME = "Elevator Home";
    public final static String SCORE_L2 = "Score L2";
    public final static String SCORE_L3 = "Score L3";
    public final static String SCORE_L4 = "Score L4";
    public final static String EXTEND_ALGAE_REMOVER = "Extend Algae Remover";
    public final static String RETRACT_ALGAE_REMOVER = "Retract Algae Remover";
    public final static String DEALGAE_ON = "Turn on Dealgae Wheels";
    public final static String DEALGAE_OFF = "Turn off Dealgae Wheels";
    public final static String MANUAL_ELEVATOR_UP = "Manual Elevator Up";
    public final static String MANUAL_ELEVATOR_DOWN = "Manual Elevator Down";

    // April Tag IDs
    public final static List<Integer> blueReefATags = List.of(17,18,19,20,21,22);
    public final static List<Integer> redReefATags = List.of(6,7,8,9,10,11);

    public final static List<Integer> blueSourceATags = List.of(12, 13);
    public final static List<Integer> redSourceATags = List.of(1, 2);
  
    // CANBUS IDs
    public final static int intakeWheelsMotorID = 13;
    public final static int coralScorerTilterMotorID = 14;
    public final static int cageClimberMotorId = 15;
    public final static int elevatorMotor1Id = 16;
    public final static int elevatorMotor2Id = 17;
    public final static int cageGrabberMotorId = 21;
    public final static int dealgaeFlipperId = 19;
    public final static int dealgaeWheelsId = 20;

    // DIO Ports
    public final static int coralSensorChannel = 1;
    public final static int reefSensorChannel = 2;

    // Subsystem Constants
    // Cage
    public final static double cageGrabberOpenPosition = -6.762;
    public final static double cageGrabberClosedPosition = 0.0;
    // Elevator
    public final static double elevatorMotorHomePosition = 0;
    public final static double elevatorMotorL2Position = 25.3;
    public final static double elevatorMotorL3Position = 31.9;
    public final static double elevatorMotorL4Position = 40;
    public final static double dealgaeFlipperExtendedPosition = 1;
    public final static double dealgaeFlipperRetractedPosition = 0;
    // Coral Scorer
    public final static Time coralDropTimeout = Seconds.of(3.0);
    public final static Angle scorerTilterScoringPosition = Degrees.of(205.0);
    public final static Angle scorerTilterLoadingPosition = Degrees.of(5.0);
    
    public final static double DRIVE_STICK_DEADBAND = 0.1;

    public static class Commands {
        public static final String DRIVE = "Drive";
        public static final String DRIVE_TO_POSE = "Drive to Pose";
        public static final String BRAKE = "Brake";
    }
}
