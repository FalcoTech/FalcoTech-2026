package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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
    public static final int INTAKEROLLER_MOTOR = 41;
    public static final int FEEDER_MOTOR = 30;
    public static final int HOPPERPUSH_MOTOR = 31;
    public static final int INTAKEPIVOT_MOTOR = 40;

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
    public static final String LIMELIGHT_MAIN = "limelight-main"; // Front facing Limelight 2+
    public static final String LIMELIGHT_REAR = "limelight-rear"; // Rear facing Limelight 3+
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

    public static final double SOFT_LOWER_LIMIT = -100;
    public static final double SOFT_UPPER_LIMIT = 100;
  }

  public static final class PathPlanningConstants {
    // From RobotContainer.java
    public static final double MAX_PATH_SPEED = 3.0; // m/s
    public static final double MAX_PATH_ACCELERATION = 4.0; // m/s²
    public static final double MAX_ANGULAR_SPEED = 540.0; // deg/s
    public static final double MAX_ANGULAR_ACCELERATION = Units.degreesToRadians(540); // rad/s²
  }

  public static final class FieldConstants {

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
