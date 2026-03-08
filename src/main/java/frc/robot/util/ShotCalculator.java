package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShotCalculator extends SubsystemBase {
  private final Turret turret;
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;

  // public ShotCalculator() {}

  public ShotCalculator(Turret turret, Shooter shooter, CommandSwerveDrivetrain drivetrain) {
    this.turret = turret;
    this.shooter = shooter;
    this.drivetrain = drivetrain;
  }

  public Angle getShotAngle(Distance distanceToTarget, LinearVelocity initialVelocity) {
    double v = initialVelocity.in(MetersPerSecond);
    double d = distanceToTarget.in(Meters);

    // TODO: Implement actual projectile motion calculation
    double angleRadians = 0.0;
    Angle angle = Radians.of(angleRadians);

    return Degrees.of(angle.in(Degrees));
  }

  public Pose3d getPose3dRelativetoRobot(Pose3d targetPose) {
    // this function should check the target pose based on what alliance we are on and where the
    // robot is currently located
    // should also account for avoiding the net
    // might need to split this into multiple functions for readability and modularity
    // TODO: Implement relative pose calculation
    Pose3d relativePose = new Pose3d(); // Placeholder for actual relative pose calculation
    return relativePose;
  }

  public boolean shotGating() {
    double tolerance = 2;
    return turret.isNearAngle(Degrees.of(getIdealTurretAngle()), Degrees.of(tolerance));
  }

  public Angle getCompensatedShotAngle() {
    // This function should calculate a compensated shot angle based on the current robot pose,
    // target pose, and calculated shot angle
    // It should account for any necessary adjustments to the shot angle based on the robot's
    // position and orientation
    return Degrees.of(0); // Placeholder for actual compensated shot angle calculation
  }

  public LinearVelocity getCompensatedShotVelocity() {
    // This function should calculate a compensated shot velocity based on the current robot pose,
    // target pose, and calculated shot angle
    // It should account for any necessary adjustments to the shot velocity based on the robot's
    // position and orientation
    return MetersPerSecond.of(0); // Placeholder for actual compensated shot velocity calculation
  }

  public double getShotViabilityScale() {
    // This function should calculate a scale from 0 to 1 representing how viable the shot is based
    // on the current robot pose, target pose, and calculated shot angle
    // It should return a value between 0 and 1, where 1 represents a highly viable shot and 0
    // represents an unviable shot
    return 0.0; // Placeholder for actual shot viability scale calculation
  }

  public double getRobotHeading() {
    // Get the current heading of the robot from -180 to 180
    return drivetrain.getState().Pose.getRotation().getDegrees();
  }

  public double getAngleToHub() {
    double getX = drivetrain.getState().Pose.getX();
    double getY = drivetrain.getState().Pose.getY();

    Translation2d robotToGoalVector =
        new Translation2d(
            4.59 - getX, // What are these magic numbers? Center of Hub?
            4.03 - getY);

    return robotToGoalVector.getAngle().getDegrees();
  }

  public double getIdealTurretAngle() {
    double idealTurretAngle = (getAngleToHub() - getRobotHeading());
    return idealTurretAngle = MathUtil.inputModulus(idealTurretAngle, -180, 180);
  }
}
