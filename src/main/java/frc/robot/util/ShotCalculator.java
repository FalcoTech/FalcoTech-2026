package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Collection;
import java.util.List;

public class ShotCalculator extends SubsystemBase {

  public record ShooterParams(double rpm, double tof) {}

  // TUNE THIS
  private static final double LATENCY_COMP = 0.15;

  private final CommandSwerveDrivetrain drivetrain;

  private Translation2d targetLocation;

  // ── Interpolation maps ────────────────────────────────────────────────────────

  private final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_MAP =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (a, b, t) -> new ShooterParams(a.rpm + (b.rpm - a.rpm) * t, a.tof + (b.tof - a.tof) * t));

  private final InterpolatingDoubleTreeMap VELOCITY_DISTANCE_MAP = new InterpolatingDoubleTreeMap();

  // distance (m) → RPM, time-of-flight (s)
  {
    put(2.0, new ShooterParams(3000, 0.3));
    put(3.0, new ShooterParams(3500, 0.4));
    put(4.0, new ShooterParams(4000, 0.5));
  }

  private void put(double distance, ShooterParams params) {
    SHOOTER_MAP.put(distance, params);
    VELOCITY_DISTANCE_MAP.put(distance / params.tof, distance);
  }

  // ── Constructor ───────────────────────────────────────────────────────────────

  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    // SHOOTER_MAP.put(2.0, new ShooterParams(2800.0, 0.42));
  }

  // ── Position helpers ──────────────────────────────────────────────────────────

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

  private double getRobotHeading() {
    // Get the current heading of the robot from -180 to 180
    return drivetrain.getState().Pose.getRotation().getDegrees();
  }

  private Translation2d getCurrentPos() {
    return drivetrain.getState().Pose.getTranslation();
  }

  private ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getState().Speeds;
  }

  private Translation2d getRobotVelocityAsTrans() {
    ChassisSpeeds robotSpeed = getChassisSpeeds();
    return new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
  }

  private Translation2d getFuturePos() {
    return getCurrentPos().plus(getRobotVelocityAsTrans().times(LATENCY_COMP));
  }
  

  // ── Shot calculation ──────────────────────────────────────────────────────────

  private Translation2d getRobotToTargetVector() {
    return getEffectiveTarget().minus(getFuturePos());
  }

  private double getDistanceToTarget() {
    return getRobotToTargetVector().getNorm();
  }

  private Translation2d getTargetVelocityVec() {
    Translation2d toTarget = getRobotToTargetVector();
    double distance = toTarget.getNorm();
    double baselineVelocity = distance / SHOOTER_MAP.get(distance).tof;
    return toTarget.div(distance).times(baselineVelocity);
  }

  private Translation2d getShotVelocity() {
    return getTargetVelocityVec().minus(getRobotVelocityAsTrans());
  }

  private double getRequiredVelocity() {
    return getShotVelocity().getNorm();
  }

  private double getEffectiveDistance() {
    return VELOCITY_DISTANCE_MAP.get(getRequiredVelocity());
  }

  // ── Public API ────────────────────────────────────────────────────────────────

  public double getAngleToTarget() {
    // return getRobotToTargetVector().getAngle().getDegrees();
    return getShotVelocity().getAngle().getDegrees();
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

  public AngularVelocity getIdealShooterVelocity() {
    double rpm = SHOOTER_MAP.get(getEffectiveDistance()).rpm;
    return RPM.of(rpm);
    // AngularVelocity idealAngularVelocity = RPM.of(3000);
    // return idealAngularVelocity;
  }

  public double getShotViabilityScale() {
    // This function should calculate a scale from 0 to 1 representing how viable the shot is based
    // on the current robot pose, target pose, and calculated shot angle
    // It should return a value between 0 and 1, where 1 represents a highly viable shot and 0
    // represents an unviable shot
    return 0.0; // Placeholder for actual shot viability scale calculation
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

  // ── Periodic ─────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ideal Turret Angle", getIdealTurretAngle().in(Degrees));
    SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    // SmartDashboard.putNumber("", getIdealShooterVelocity())
  }
}
