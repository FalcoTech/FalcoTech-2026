package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Collection;
import java.util.List;

public class ShotCalculator extends SubsystemBase {

  private final CommandSwerveDrivetrain drivetrain;

  private Translation2d targetLocation;

  private InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    shooterMap.put(2.0, 1500.0);
  }

  private Translation2d getEffectiveTarget() {
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

  // TODO: MOVE Shot Checking to better location
  // public boolean shotGating() {
  //   // Report if the turret is near the ideal angle and also check shooter speed
  //   double angleTolerance = 2; // Degrees
  //   Boolean angleGood =
  //       turret
  //           .isNearAngle(Degrees.of(getIdealTurretAngle()), Degrees.of(angleTolerance))
  //           .getAsBoolean();
  //   double velocityTolerance = 100; // RPMs
  //   Boolean velocityGood =
  //       shooter
  //           .isNearVelocity(RPM.of(getIdealShooterVelocity()), RPM.of(velocityTolerance))
  //           .getAsBoolean();

  //   return angleGood && velocityGood;
  // }

  private AngularVelocity getIdealShooterVelocity() {
    // TODO Add actual math using LUT
    AngularVelocity idealAngularVelocity = RPM.of(3000);
    return idealAngularVelocity;
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

  public Translation2d getRobotToTargetVector() {
    return getEffectiveTarget().minus(drivetrain.getState().Pose.getTranslation());
  }

  public double getDistanceToTarget() {
    return getRobotToTargetVector().getNorm();
  }

  public double getAngleToTarget() {
    // double getX = drivetrain.getState().Pose.getX();
    // double getY = drivetrain.getState().Pose.getY();

    // Translation2d robotToHubVector =
    //     new Translation2d(
    //         4.59 - getX, // What are these magic numbers? Center of Hub?
    //         4.03 - getY);

    return getRobotToTargetVector().getAngle().getDegrees();
  }

  public double Clamp(double value, double lowerBound, double upperBound) {
    return Math.max(lowerBound, Math.min(upperBound, value));
  }

  /**
   * @return Turret Angle in WPILib Blue Coordinate?
   */
  public Angle getIdealTurretAngle() {
    double idealTurretAngle = (getAngleToTarget() - getRobotHeading());
    double wrappedTurretAngle = MathUtil.inputModulus(idealTurretAngle, -180, 180);

    // TODO: HIGH PRIORITY - Move this to the actual named command that will handling turret
    // alignment
    // if ((wrappedTurretAngle < -50) || (wrappedTurretAngle > 50)) {
    //   turret.stop();
    //   return Clamp(wrappedTurretAngle, -50, 50);
    // } else {
    //   return Clamp(wrappedTurretAngle, -50, 50);
    // }
    return Degrees.of(wrappedTurretAngle);
  }

  public double getShooterRpm() {
    return shooterMap.get(getDistanceToTarget());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Ideal Turret Angle", getIdealTurretAngle().in(Degrees));
    SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    // SmartDashboard.putNumber("", getIdealShooterVelocity())
  }
}
