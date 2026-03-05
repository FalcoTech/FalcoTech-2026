package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;

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
    public static final int FLYWHEEL_MOTOR_LEFT = 94;
    public static final int FLYWHEEL_MOTOR_RIGHT = 95;
    public static final int TURRET_MOTOR = 20;

    // Intake & Feeder Motors
    public static final int INTAKE_ROLLER_MOTOR = 96;
    public static final int INTAKE_ARM_MOTOR = 93;
    public static final int FEEDER_MOTOR = 97;

    // Climb Motors
    public static final int CLIMB_ELEVATOR_MOTOR = 98;
    public static final int CLIMB_PIVOT_MOTOR = 99;
    public static final int CLIMB_Arm_MOTOR = 0;

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
    public static final String LIMELIGHT_NAME = "primary"; // Front facing camera
    public static final String LIMELIGHT2_NAME = "secondary"; // Rear facing camera
    public static final boolean USE_LIMELIGHT = true;
    // Don't use vision measurements when rotating faster than this
    public static final double VISION_OMEGA_CUTOFF_RPS = 2.0;
  }

  public static final class IntakeConstants {}

  public static final class ClimbConstants {
    public static final Current LIFTOFF_THRESHOLD =
        Amps.of(20); // Number of amps seen with Robot weight

    // Elevator motor closed loop controller
    public static final double ELEVATOR_kP = 4;
    public static final double ELEVATOR_kI = 0;
    public static final double ELEVATOR_kD = 0;
    public static final LinearVelocity ELEVATOR_MAX_VELOCITY = FeetPerSecond.of(0.5);
    public static final LinearAcceleration ELEVATOR_MAX_ACCELERATION = FeetPerSecondPerSecond.of(1);

    // Elevator feedforward gains (kS, kG, kV)
    public static final double ELEVATOR_FF_kS = 0;
    public static final double ELEVATOR_FF_kG = 0;
    public static final double ELEVATOR_FF_kV = 0;

    // Elevator motor hardware config
    public static final Current ELEVATOR_STATOR_CURRENT_LIMIT = Amps.of(40);
    public static final String ELEVATOR_GEARBOX_STAGES = "1:100";
    public static final String ELEVATOR_SPROCKET_STAGES = "1:4";

    // Elevator physical properties
    public static final Distance ELEVATOR_STARTING_HEIGHT = Inches.of(0.5);
    public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(6);
    public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(12);
    public static final Mass ELEVATOR_MASS = Pounds.of(12);

    public static final double ARM_PIVOT_kP = 0;

    public static final double ARM_PIVOT_kI = 0;

    public static final double ArmPivot_FF_kG = 0;

    public static final double ARM_PIVOT_kD = 0;

    public static final double ArmPivot_FF_kV = 0;

    public static final LinearVelocity ARM_PIVOT_MAX_VELOCITY = null;

    public static final LinearAcceleration ARM_PIVOT_MAX_ACCELERATION = null;

    public static final String Arm_SPROCKET_STAGES = null;

    public static final String Arm_GEARBOX_STAGES = null;

    public static final Current ARM_STATOR_CURRENT_LIMIT = null;

    public static final Mass Arm_MASS = Pounds.of(5);

    public static final Angle Arm_MIN_Position = Degrees.of(0);

    public static final Angle Arm_MAX_Position = Degrees.of(90);

    public static double ArmPivot_FF_kS;
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
