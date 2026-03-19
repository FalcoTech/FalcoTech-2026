package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import java.util.Collection;
import java.util.List;

public class ShotCalculator extends SubsystemBase {
  private static final Turret turret = RobotContainer.turret;
  private static final Shooter shooter = RobotContainer.shooter;
  private static final CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

  private static Translation2d targetLocation;

  private static InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

  public ShotCalculator(Turret turret, Shooter shooter, CommandSwerveDrivetrain drivetrain) {
    

        
    shooterMap.put(1.77, 3000.0);
    shooterMap.put(2.18, 3500.0);
    shooterMap.put(3.01, 4000.0);
  }

  public static Angle getShotAngle(Distance distanceToTarget, LinearVelocity initialVelocity) {
    double v = initialVelocity.in(MetersPerSecond);
    double d = distanceToTarget.in(Meters);

    // TODO: This probably can be scrapped
    double angleRadians = 0.0;
    Angle angle = Radians.of(angleRadians);

    return Degrees.of(angle.in(Degrees));
  }

  private static Translation2d getEffectiveTarget() {
    // Update the target pose based on what FieldZone the Robot currently is in
    Pose2d robotPose = drivetrain.getState().Pose;
    // Convert robot Pose to blue coords when on red alliance so blue defined math is correct
    Translation2d robotPositionBlue;
    if (Constants.isRedAlliance()) {
      robotPositionBlue = FlippingUtil.flipFieldPosition(robotPose.getTranslation());
    } else robotPositionBlue = robotPose.getTranslation();

    if (FieldConstants.allianceZone.contains(robotPositionBlue)) {
      targetLocation = FieldConstants.HUB_TARGET;
    } else {
      Collection<Translation2d> locations =
          List.of(FieldConstants.OUTPOST_SIDE_TARGET, FieldConstants.DEPOT_SIDE_TARGET);
      targetLocation = robotPositionBlue.nearest(locations);
    }

    // Targets are defined in blue-alliance coords; flip to absolute field coords on red alliance
    // so that callers can use robot pose (always absolute) directly for vector math.
    if (Constants.isRedAlliance()) {
      return FlippingUtil.flipFieldPosition(targetLocation);
    }
    return targetLocation;
  }

  public static boolean shotGating() {
    // Report if the turret is near the ideal angle and also check shooter speed
    double angleTolerance = 2; // Degrees
    Boolean angleGood =
        turret
            .isNearAngle(Degrees.of(getIdealTurretAngle()), Degrees.of(angleTolerance))
            .getAsBoolean();
    double velocityTolerance = 100; // RPMs
    Boolean velocityGood =
        shooter
            .isNearVelocity(RPM.of(getIdealShooterVelocity()), RPM.of(velocityTolerance))
            .getAsBoolean();

    return angleGood && velocityGood;
  }

  private static double getIdealShooterVelocity() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getIdealShooterVelocity'");
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

  public static double getRobotHeading() {
    // Get the current heading of the robot from -180 to 180
    return drivetrain.getState().Pose.getRotation().getDegrees();
  }

  public static Translation2d getRobotToTargetVector() {
    return getEffectiveTarget().minus(drivetrain.getState().Pose.getTranslation());
  }

  public static double getDistanceToTarget() {
    return getRobotToTargetVector().getNorm();
  }

  public static double getAngleToTarget() {
    // double getX = drivetrain.getState().Pose.getX();
    // double getY = drivetrain.getState().Pose.getY();

    // Translation2d robotToHubVector =
    //     new Translation2d(
    //         4.59 - getX, // What are these magic numbers? Center of Hub?
    //         4.03 - getY);

    return getRobotToTargetVector().getAngle().getDegrees();
  }

  public static double Clamp(double value, double lowerBound, double upperBound) {
    return Math.max(lowerBound, Math.min(upperBound, value));
  }

  public static double getIdealTurretAngle() {
    double idealTurretAngle = (getAngleToTarget() - getRobotHeading());
    double wrappedTurretAngle = MathUtil.inputModulus(idealTurretAngle, -180, 180);

    if ((wrappedTurretAngle < TurretConstants.SOFT_LOWER_LIMIT) || (wrappedTurretAngle > TurretConstants.SOFT_UPPER_LIMIT)) {
      return Clamp(wrappedTurretAngle, TurretConstants.SOFT_LOWER_LIMIT, TurretConstants.SOFT_UPPER_LIMIT);
    } else {
      return Clamp(wrappedTurretAngle, TurretConstants.SOFT_LOWER_LIMIT, TurretConstants.SOFT_UPPER_LIMIT);
    }
  }
  public static double getIdealShooterSpeed(){
    return shooterMap.get(getDistanceToTarget());
  }

  public static double getShooterRpm() {
    return shooterMap.get(getDistanceToTarget());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Ideal Turret Angle", getIdealTurretAngle());
    SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    // SmartDashboard.putNumber("", getIdealShooterVelocity())
  }
}
