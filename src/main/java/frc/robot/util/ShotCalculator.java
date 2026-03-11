package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ShotCalculator extends SubsystemBase {
  // private final CommandSwerveDrivetrain drivetrain;
  // private final Shooter shooter;
  // private final Turret turret;

  public ShotCalculator(Turret turret, Shooter shooter, CommandSwerveDrivetrain drivetrain) {
    this.turret = turret;
    this.shooter = shooter;
    this.drivetrain = drivetrain;
  }
  // private final Vision vision;
  public ShotCalculator() {}

  // public ShotCalculator(Turret turret, Shooter shooter, CommandSwerveDrivetrain drivetrain) {
  //   this.turret = turret;
  //   this.shooter = shooter;
  //   this.drivetrain = drivetrain;
  //   // this.vision = vision;
  // }

  public static Angle getShotAngle(Distance distanceToTarget, LinearVelocity initialVelocity) {
    double v = initialVelocity.in(MetersPerSecond);
    double d = distanceToTarget.in(Meters);

    // TODO: Implement actual projectile motion calculation things
    double angleRadians = 0.0;
    Angle angle = Radians.of(angleRadians);

    return Degrees.of(angle.in(Degrees));
  }

  public static Pose3d getPose3dRelativetoRobot(Pose3d targetPose) {
    // this function should check the target pose based on what alliance we are on and where the
    // robot is currently located
    // should also account for avoiding the net
    // might need to split this into multiple functions for readability and modularity
    // TODO: Implement relative pose calculation
    Pose3d relativePose = new Pose3d(); // Placeholder for actual relative pose calculation
    return relativePose;
  }

  public static boolean shotGating() {
    // This function should determine if the shot is viable based on the current robot pose, target
    // pose, and calculated shot angle
    // It should return true if the shot is viable and false if it is not
    return false;
  }

  public static Angle getCompensatedShotAngle() {
    // This function should calculate a compensated shot angle based on the current robot pose,
    // target pose, and calculated shot angle
    // It should account for any necessary adjustments to the shot angle based on the robot's
    // position and orientation
    return Degrees.of(0); // Placeholder for actual compensated shot angle calculation
  }

  public static LinearVelocity getCompensatedShotVelocity() {
    // This function should calculate a compensated shot velocity based on the current robot pose,
    // target pose, and calculated shot angle
    // It should account for any necessary adjustments to the shot velocity based on the robot's
    // position and orientation
    return MetersPerSecond.of(0); // Placeholder for actual compensated shot velocity calculation
  }

  public static double getShotViabilityScale() {
    // This function should calculate a scale from 0 to 1 representing how viable the shot is based
    // on the current robot pose, target pose, and calculated shot angle
    // It should return a value between 0 and 1, where 1 represents a highly viable shot and 0
    // represents an unviable shot
    return 0.0; // Placeholder for actual shot viability scale calculation
  }

  public static double getGyroHeading() {
    return (RobotContainer.drivetrain.getState().RawHeading.getDegrees() + 360000) % 360;
  }

  public static double getAngleToHub() {
    double getX = RobotContainer.drivetrain.getState().Pose.getX();
    double getY = RobotContainer.drivetrain.getState().Pose.getY();

    Translation2d robotToGoalVector = new Translation2d(4.59 - getX, 4.03 - getY);

    double rawAngle = (robotToGoalVector.getAngle().getDegrees()) % 360;
    if (rawAngle < 0) {
      return rawAngle + 360;
    } else {
      return rawAngle;
    }
  }

  public static double getTurretAngle() {
    double offset = 90;
    double rawAngle = (getAngleToHub() - getGyroHeading() + offset) % 360;
    double filteredAngle;
    if (rawAngle < 0) {
      filteredAngle = rawAngle + 360;
    } else {
      filteredAngle = rawAngle;
    }
    ;

    if (filteredAngle > 0 && filteredAngle < 200) {
      SmartDashboard.putBoolean("Turret Possible Shot", true);
      return filteredAngle;

    } else {
      SmartDashboard.putBoolean("Turret Possible Shot", false);
      return RobotContainer.turret.getAngle().in(Degrees);
    }
  }
}
