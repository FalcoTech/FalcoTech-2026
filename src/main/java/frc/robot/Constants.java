package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    public static final int INTAKEPIVOT_MOTOR = 31;
    public static final int SPINNERINDEXLEFT_MOTOR = 46;
    public static final int SPINNERINDEXRIGHT_MOTOR = 45;

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

  public static final class HoodConstants {
    public static final double HOOD_UP = 0.4;
    public static final double HOOD_DOWN = 0;
  }

  /**
   * Centralized current limits for all motor controllers. Adjust values here, not in subsystems.
   */
  public static final class CurrentLimits {
    // Swerve Drive (TalonFX / Kraken X60)
    public static final Current SWERVE_DRIVE_STATOR = Amps.of(30);
    public static final Current SWERVE_DRIVE_SUPPLY = Amps.of(30);
    // Swerve Steer (TalonFX)
    public static final Current SWERVE_STEER_STATOR = Amps.of(30);
    public static final Current SWERVE_STEER_SUPPLY = Amps.of(20);
    // SpinnerIndex (TalonFX / Kraken X60, dual motor)
    public static final Current SPINNER_INDEX_STATOR = Amps.of(20);
    public static final Current SPINNER_INDEX_SUPPLY = Amps.of(30);
    // IntakePivot (SparkMax / NEO)
    public static final Current INTAKE_PIVOT_STATOR = Amps.of(80);
    // Turret (TalonFX / Falcon 500)
    public static final Current TURRET_STATOR = Amps.of(20);
    // Shooter (SparkMax / dual NEO)
    public static final Current SHOOTER_STATOR = Amps.of(40);
    // IntakeRoller (SparkFlex)
    public static final Current INTAKE_ROLLER_SMART = Amps.of(40);
    // Feeder (SparkMax / NEO)
    public static final Current FEEDER_SMART = Amps.of(40);
  }

  public static final class TurretConstants {
    // CCW is positive in WPILib (think Unit Circle)
    public static final Angle HARD_COUNTER_CLOCKWISE_LIMIT = Degrees.of(160);
    public static final Angle HARD_CLOCKWISE_LIMIT = Degrees.of(-160);

    public static final Angle DEADZONE = Degrees.of(10);
    public static final double SOFT_LOWER_LIMIT = HARD_CLOCKWISE_LIMIT.plus(DEADZONE).in(Degrees);
    public static final double SOFT_UPPER_LIMIT =
        HARD_COUNTER_CLOCKWISE_LIMIT.minus(DEADZONE).in(Degrees);
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
    public static final Rectangle2d ALLIANCE_RECTANGLE2D =
        new Rectangle2d(new Translation2d(-.5, -.5), new Translation2d(4.278, 8.5));
    public static final Rectangle2d NEUTRAL_RECTANGLE2D =
        new Rectangle2d(new Translation2d(4.278, -.5), new Translation2d(12.117, 8.5));

    public static final double TRENCH_SLOW_RADIUS = 1.5;
    // Slow down when within range of trench TODO: tune this value

    public static final Ellipse2d BLUEOUTPOST_ELLIPSE2D =
        new Ellipse2d(new Translation2d(4.635, 7.450), TRENCH_SLOW_RADIUS);
    public static final Ellipse2d BLUEHUMAN_ELLIPSE2D =
        new Ellipse2d(new Translation2d(4.635, .635), TRENCH_SLOW_RADIUS);
    public static final Ellipse2d REDOUTPOST_ELLIPSE2D =
        new Ellipse2d(
            FlippingUtil.flipFieldPosition(BLUEOUTPOST_ELLIPSE2D.getCenter().getTranslation()),
            TRENCH_SLOW_RADIUS);
    public static final Ellipse2d REDHUMAN_ELLIPSE2D =
        new Ellipse2d(
            FlippingUtil.flipFieldPosition(BLUEHUMAN_ELLIPSE2D.getCenter().getTranslation()),
            TRENCH_SLOW_RADIUS);
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
