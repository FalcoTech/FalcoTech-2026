package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.FieldZone;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int BLINKIN_PWM_PORT = 9;
  public static final Voltage VOLTAGE_COMP = Volts.of(12);

  public static final class CAN_IDs {
    // Drivetrain IDs are handled by TunerConstants and are omitted here

    // Shooter Motors
    public static final int TURRET_MOTOR = 20;
    public static final int FLYWHEEL_MOTOR_LEFT = 21;
    public static final int FLYWHEEL_MOTOR_RIGHT = 22;

    // Intake & Feeder Motors
    public static final int INTAKEROLLER_MOTOR = 80;
    public static final int FEEDER_MOTOR = 30;
    public static final int HOPPERPUSH_MOTOR = 31;

    // Climb Motors
    public static final int CLIMB_ELEVATOR_MOTOR = 98;
    public static final int CLIMB_PIVOT_MOTOR = 99;
    public static final int CLIMB_Arm_MOTOR = 99;
    public static final int INTAKESLIDE_MOTOR = 100;

    // Sensors and other devices could also be added here
  }

  public static final class ControllerConstants {
    public static final int PILOT_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;
    public static final double DEADBAND = 0.1;
    public static final double TRIGGER_THRESHOLD = 0.2;
  }

  public static final class DrivetrainConstants {
    // These could replace some values in TunerConstants or provide fallbacks
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.96;
    public static final double MAX_ANGULAR_RATE_RADIANS_PER_SECOND =
        0.75 * 2 * Math.PI; // 3/4 rotation per second
  }

  public static final class VisionConstants {
    public static final String LIMELIGHT_MAIN = "limelight-main"; // Front facing Limelight 3
    public static final String LIMELIGHT_REAR = "limelight-rear"; // Rear facing Limelight 2+
    public static final boolean USE_LIMELIGHT = true;
    // Don't use vision measurements when rotating faster than this
    public static final double VISION_OMEGA_CUTOFF_RPS = 2.0;
    // MegaTag2 standard deviations (trust XY from vision, trust rotation from Pigeon)
    public static final double MEGATAG2_XY_STDDEV = 0.5;
    public static final double MEGATAG2_ROTATION_STDDEV = 9999999;
  }

  public static final class IntakeConstants {}

  public static final class TurretConstants {
    // CCW is positive in WPILib (think Unit Circle)
    public static final Angle HARD_COUNTER_CLOCKWISE_LIMIT = Degrees.of(140);
    public static final Angle HARD_CLOCKWISE_LIMIT = Degrees.of(-140);
  }

  public static final class ClimbConstants {
    public static final Current LIFTOFF_THRESHOLD =
        Amps.of(20); // Number of amps seen with Robot weight

    // Elevator motor closed loop controller
    public static final double ELEVATOR_kP = 0;
    public static final double ELEVATOR_kI = 0;
    public static final double ELEVATOR_kD = 0;
    public static final LinearVelocity ELEVATOR_MAX_VELOCITY = FeetPerSecond.of(2);
    public static final LinearAcceleration ELEVATOR_MAX_ACCELERATION = FeetPerSecondPerSecond.of(4);

    // Elevator feedforward gains (kS, kG, kV)
    public static final double ELEVATOR_FF_kS = 0;
    public static final double ELEVATOR_FF_kG = 0.0622;
    public static final double ELEVATOR_FF_kV = 10;

    // Elevator motor hardware config
    public static final Current ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final String ELEVATOR_GEARBOX_STAGES = "100:1";
    public static final String ELEVATOR_SPROCKET_STAGES = "1:1";

    // Elevator physical properties
    public static final Distance ELEVATOR_STARTING_HEIGHT = Inches.of(2.5);
    public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(2.5);
    public static final Distance ELEVATOR_MAX_HEIGHT =
        ELEVATOR_MIN_HEIGHT.plus(Inches.of(5.25)); // Travel Distance
    public static final Mass ELEVATOR_MASS = Pounds.of(12);

    // Arm motor closed loop controller
    public static final double ARM_kP = 0;
    public static final double ARM_kI = 0;
    public static final double ARM_kD = 0;
    public static final AngularVelocity ARM_MAX_VELOCITY = DegreesPerSecond.of(50);
    public static final AngularAcceleration ARM_MAX_ACCELERATION = DegreesPerSecondPerSecond.of(25);

    // Arm feedforward gains (kS, kG, kV)
    public static final double ARM_FF_kS = 0;
    public static final double ARM_FF_kG = 0;
    public static final double ARM_FF_kV = 0;

    // Arm motor hardware config
    public static final Current ARM_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final String ARM_GEARBOX_STAGES = "1:100";
    public static final String ARM_SPROCKET_STAGES = "1:4";

    // Arm physical properties
    public static final Angle ARM_MIN_Position = Degrees.of(0);
    public static final Angle ARM_MAX_Position = Degrees.of(90);
    public static final Mass ARM_MASS = Pounds.of(2.5);
  }

  public static final class PathPlanningConstants {
    // From RobotContainer.java
    public static final double MAX_PATH_SPEED = 3.0; // m/s
    public static final double MAX_PATH_ACCELERATION = 4.0; // m/s²
    public static final double MAX_ANGULAR_SPEED = 540.0; // deg/s
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(540); // rad/s²

    // Field positions
    // public static final Pose2d LEFT_FEEDER_POSE = new Pose2d(1.14, 6.93,
    // Rotation2d.fromDegrees(127.16));
    // public static final Pose2d ALGAE_SCORE_POSE = new Pose2d(5.98, .58,
    // Rotation2d.fromDegrees(-90));
  }

  public static final class FieldConstants {

    // Distance constants (in meters)
    // public static final double CORAL_FORWARD_DISTANCE = -0.4572;  // 18 inches away from tag
    // public static final double CORAL_LATERAL_DISTANCE = 0.1524;  // 6 inches for left/right
    // offset

    // // Tag-specific offset constants
    // public static final double CORAL_STATION_FORWARD_OFFSET = -0.610;  //
    // public static final double CORAL_STATION_LATERAL_OFFSET = 0.610;  // 2ft
    // public static final double ALGAE_REEF_LATERAL_OFFSET = 0.0;  // No lateral offset
    // public static final double ALGAE_PROC_FORWARD_OFFSET = -0.5842;  // 23 in
    // public static final double BARGE_FORWARD_OFFSET = -1.0;  // 1 meter

    // All positions are based on blue side origin and must be flipped when needed
    public static final Translation2d OUTPOST_SIDE_TARGET = new Translation2d(2, 2);
    public static final Translation2d DEPOT_SIDE_TARGET = new Translation2d(2, 6);
    public static final Translation2d HUB_TARGET = new Translation2d(4.59, 4.03);
    public static final FieldZone allianceZone = new FieldZone(-.5, 4.278, -.5, 8.5);
    public static final FieldZone neutralZone = new FieldZone(4.278, 12.117, -.5, 8.5);
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
