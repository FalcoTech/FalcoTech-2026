package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// spotless:off                                                                                                                 
//  ▄    ▄                 ▄           ▄    ▄        ▀▀█                                                           
//  ██  ██  ▄▄▄    ▄▄▄   ▄▄█▄▄         ▀▄  ▄▀  ▄▄▄     █    ▄   ▄   ▄▄▄    ▄▄▄                                     
//  █ ██ █ █▀ ▀█  █   ▀    █            █  █  ▀   █    █    █   █  █▀  █  █   ▀                                    
//  █ ▀▀ █ █   █   ▀▀▀▄    █            ▀▄▄▀  ▄▀▀▀█    █    █   █  █▀▀▀▀   ▀▀▀▄                                    
//  █    █ ▀█▄█▀  ▀▄▄▄▀    ▀▄▄           ██   ▀▄▄▀█    ▀▄▄  ▀▄▄▀█  ▀█▄▄▀  ▀▄▄▄▀                                    
                                                                                                                
                                                                                                                
                                                                                                                
//    ▄▄                        ▄▄▄▄▄  ▀▀█                         █             ▀▀█        █                      
//    ██    ▄ ▄▄   ▄▄▄          █   ▀█   █     ▄▄▄    ▄▄▄    ▄▄▄   █ ▄▄    ▄▄▄     █     ▄▄▄█   ▄▄▄    ▄ ▄▄   ▄▄▄  
//   █  █   █▀  ▀ █▀  █         █▄▄▄█▀   █    ▀   █  █▀  ▀  █▀  █  █▀  █  █▀ ▀█    █    █▀ ▀█  █▀  █   █▀  ▀ █   ▀ 
//   █▄▄█   █     █▀▀▀▀         █        █    ▄▀▀▀█  █      █▀▀▀▀  █   █  █   █    █    █   █  █▀▀▀▀   █      ▀▀▀▄ 
//  █    █  █     ▀█▄▄▀         █        ▀▄▄  ▀▄▄▀█  ▀█▄▄▀  ▀█▄▄▀  █   █  ▀█▄█▀    ▀▄▄  ▀█▄██  ▀█▄▄▀   █     ▀▄▄▄▀ 
// spotless:on

public final class Constants {
  public static final int BLINKIN_PWM_PORT = 9;

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
    public static final String LIMELIGHT_MAIN = "primary"; // Front facing Limelight 3
    public static final String LIMELIGHT_REAR = "secondary"; // Rear facing Limelight 2+
    public static final boolean USE_LIMELIGHT = true;
    // Don't use vision measurements when rotating faster than this
    public static final double VISION_OMEGA_CUTOFF_RPS = 2.0;
    // MegaTag2 standard deviations (trust XY from vision, trust rotation from Pigeon)
    public static final double MEGATAG2_XY_STDDEV = 0.5;
    public static final double MEGATAG2_ROTATION_STDDEV = 9999999;
  }

  public static final class IntakeConstants {}

  public static final class ClimbConstants {}

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

  public static final class AlignmentConstants {
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
  }
}
