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
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Collection;
import java.util.List;

public class ShotCalculator extends SubsystemBase {

  public record ShooterParams(double rpm, double tof) {}

  // ── Viability tuning ──────────────────────────────────────────────────────────

  // Degrees before the hard limit where the turret angle score begins to drop.
  private static final double TURRET_APPROACH_MARGIN_DEG = 10.0;
  // Distance margin (m) outside the SHOOTER_MAP bounds where score ramps to 0.
  private static final double DISTANCE_MARGIN_M = 0.5;
  // Lateral speed (m/s) at which the lateral-speed score reaches 0.
  private static final double LATERAL_SPEED_THRESHOLD_MPS = 1.5;

  // TUNE THIS
  private static final double LATENCY_COMP = 0.15;

  private final CommandSwerveDrivetrain drivetrain;

  private static Translation2d targetLocation;

  // ── Interpolation maps ────────────────────────────────────────────────────────

  private final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_MAP =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (a, b, t) -> new ShooterParams(a.rpm + (b.rpm - a.rpm) * t, a.tof + (b.tof - a.tof) * t));

        
    shooterMap.put(1.77, 3000.0);
    shooterMap.put(2.18, 3500.0);
    shooterMap.put(3.01, 4000.0);
    shooterMap.put(4.0, 5000.0);

  }

  private void put(double distance, ShooterParams params) {
    SHOOTER_MAP.put(distance, params);
    VELOCITY_DISTANCE_MAP.put(distance / params.tof, distance);
    if (distance < shooterMapMinDistance) shooterMapMinDistance = distance;
    if (distance > shooterMapMaxDistance) shooterMapMaxDistance = distance;
  }

  // ── Constructor ───────────────────────────────────────────────────────────────

  public ShotCalculator(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
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
    return getTargetVelocityVec().plus(getRobotVelocityAsTrans());
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
    return Degrees.of(wrappedTurretAngle);
  }

    if ((wrappedTurretAngle < TurretConstants.SOFT_LOWER_LIMIT) || (wrappedTurretAngle > TurretConstants.SOFT_UPPER_LIMIT)) {
      return Clamp(wrappedTurretAngle, TurretConstants.SOFT_LOWER_LIMIT, TurretConstants.SOFT_UPPER_LIMIT);
    } else {
      return Clamp(wrappedTurretAngle, TurretConstants.SOFT_LOWER_LIMIT, TurretConstants.SOFT_UPPER_LIMIT);
    }
  }

  /**
   * Returns a 0→1 scale representing how viable the current shot is. All three factors must be
   * favorable — any single disqualifier collapses the score toward zero.
   *
   * <ul>
   *   <li>Turret angle: score drops from 1→0 across the approach margin before the hard limit.
   *   <li>Distance: score is 1 inside the SHOOTER_MAP range, ramps to 0 outside the margin.
   *   <li>Lateral speed: score is 1 at rest, drops to 0 at {@link #LATERAL_SPEED_THRESHOLD_MPS}.
   * </ul>
   */
  public double getShotViabilityScale() {
    return getTurretAngleScore() * getDistanceScore() * getLateralSpeedScore();
  }

  private double getTurretAngleScore() {
    double absAngle = Math.abs(getIdealTurretAngle().in(Degrees));
    double hardLimit = TurretConstants.HARD_COUNTER_CLOCKWISE_LIMIT.in(Degrees);
    return MathUtil.clamp((hardLimit - absAngle) / TURRET_APPROACH_MARGIN_DEG, 0.0, 1.0);
  }

  private double getDistanceScore() {
    double distance = getDistanceToTarget();
    if (distance >= shooterMapMinDistance && distance <= shooterMapMaxDistance) return 1.0;
    if (distance < shooterMapMinDistance)
      return MathUtil.clamp(
          (distance - (shooterMapMinDistance - DISTANCE_MARGIN_M)) / DISTANCE_MARGIN_M, 0.0, 1.0);
    return MathUtil.clamp(
        ((shooterMapMaxDistance + DISTANCE_MARGIN_M) - distance) / DISTANCE_MARGIN_M, 0.0, 1.0);
  }

  private double getLateralSpeedScore() {
    Translation2d toTarget = getRobotToTargetVector();
    double norm = toTarget.getNorm();
    if (norm < 0.01) return 1.0;
    Translation2d unitToTarget = toTarget.div(norm);
    Translation2d robotVel = getRobotVelocityAsTrans();
    double radial = robotVel.getX() * unitToTarget.getX() + robotVel.getY() * unitToTarget.getY();
    double velNormSq = robotVel.getX() * robotVel.getX() + robotVel.getY() * robotVel.getY();
    double lateral = Math.sqrt(Math.max(0.0, velNormSq - radial * radial));
    return MathUtil.clamp(1.0 - lateral / LATERAL_SPEED_THRESHOLD_MPS, 0.0, 1.0);
  }

  // ── Periodic ─────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ideal Turret Angle", getIdealTurretAngle().in(Degrees));
    SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    SmartDashboard.putNumber("Shot Viability", getShotViabilityScale());
  }
}
