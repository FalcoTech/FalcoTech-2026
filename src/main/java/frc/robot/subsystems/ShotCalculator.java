package frc.robot.subsystems;

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
import java.util.Collection;
import java.util.List;

/**
 * Calculates optimal turret angle and shooter RPM based on the robot's field pose, velocity, and an
 * interpolation table of distance-to-shot-parameters.
 *
 * <p>Target selection is automatic: when the robot is inside the alliance zone it aims at the hub;
 * otherwise it picks the nearest non-hub target (outpost or depot). All target coordinates are
 * defined in blue-alliance space and flipped at runtime for red.
 *
 * <p>Robot velocity is compensated for both latency ({@code LATENCY_COMP}) and ballistic lead, so
 * the turret and shooter adjust as the robot moves.
 */
public class ShotCalculator extends SubsystemBase {

  /**
   * Holds the interpolated shot parameters for a given distance.
   *
   * @param rpm flywheel speed in revolutions per minute
   * @param tof estimated time-of-flight in seconds
   */
  public record ShooterParams(double rpm, double tof) {}

  // ── Viability tuning (live-tunable via SmartDashboard) ─────────────────────

  // Degrees before the hard limit where the turret angle score begins to drop.
  private double turretApproachMarginDeg = 10.0;
  // Distance margin (m) outside the SHOOTER_MAP bounds where score ramps to 0.
  private double distanceMarginM = 0.5;
  // Lateral speed (m/s) at which the lateral-speed score reaches 0.
  private double lateralSpeedThresholdMPS = 1.5;

  // TUNE THIS
  private double latencyComp = 0.15;

  private final CommandSwerveDrivetrain drivetrain;

  private Translation2d targetLocation;

  // ── Interpolation maps ────────────────────────────────────────────────────────

  private final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_MAP =
      new InterpolatingTreeMap<>(
          InverseInterpolator.forDouble(),
          (a, b, t) -> new ShooterParams(a.rpm + (b.rpm - a.rpm) * t, a.tof + (b.tof - a.tof) * t));

  private final InterpolatingDoubleTreeMap VELOCITY_DISTANCE_MAP = new InterpolatingDoubleTreeMap();

  private double shooterMapMinDistance = Double.MAX_VALUE;
  private double shooterMapMaxDistance = Double.MIN_VALUE;

  // distance (m) → RPM, time-of-flight (s) — TUNE tof values
  {
    // TODO: GET NEW DATA
    put(1.77, new ShooterParams(3000, 2.26));
    put(2.18, new ShooterParams(3500, 3.32));
    put(3.01, new ShooterParams(4000, 4.43));
    put(3.82, new ShooterParams(4500, 4.5));
    put(4.19, new ShooterParams(5000, 5.50));
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
    SmartDashboard.putNumber("Tuning/LatencyComp", latencyComp);
    // SmartDashboard.putNumber("Tuning/TurretApproachMarginDeg", turretApproachMarginDeg);
    // SmartDashboard.putNumber("Tuning/DistanceMarginM", distanceMarginM);
    // SmartDashboard.putNumber("Tuning/LateralSpeedThresholdMPS", lateralSpeedThresholdMPS);
  }

  // ── Position helpers ──────────────────────────────────────────────────────────

  private Translation2d getEffectiveTarget() {
    // Update the target pose based on what Rectangle2d the Robot currently is in
    Pose2d robotPose = drivetrain.getState().Pose;
    // Convert robot Pose to blue coords when on red alliance so blue defined math is correct
    Translation2d robotPositionBlue;
    if (Constants.isRedAlliance()) {
      robotPositionBlue = FlippingUtil.flipFieldPosition(robotPose.getTranslation());
    } else robotPositionBlue = robotPose.getTranslation();

    if (FieldConstants.ALLIANCE_RECTANGLE2D.contains(robotPositionBlue)) {
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
    return getCurrentPos().plus(getRobotVelocityAsTrans().times(latencyComp));
  }

  // ── Shot calculation ──────────────────────────────────────────────────────────

  private Translation2d getRobotToTargetVector() {
    return getEffectiveTarget().minus(getFuturePos());
  }

  public double getDistanceToTarget() {
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

  /**
   * @return the field-relative angle (degrees) from the robot to the target, with velocity lead.
   */
  public double getAngleToTarget() {
    // return getRobotToTargetVector().getAngle().getDegrees();
    // NOTE: Switch this out with the line above to disable velocity compensation and see how it
    // affects aiming while moving
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

  /** Ideal shooter wheel speed for the current effective distance. */
  public AngularVelocity getIdealShooterVelocity() {
    return RPM.of(SHOOTER_MAP.get(getEffectiveDistance()).rpm);
  }

  /** Returns true when the robot is in the alliance zone and targeting the main hub. */
  public boolean isTargetingHub() {
    getEffectiveTarget(); // ensures targetLocation is current
    return targetLocation == FieldConstants.HUB_TARGET;
  }

  /**
   * Returns a 0→1 scale representing how viable the current shot is. All three factors must be
   * favorable — any single disqualifier collapses the score toward zero.
   *
   * <ul>
   *   <li>Turret angle: score drops from 1→0 across the approach margin before the hard limit.
   *   <li>Distance: score is 1 inside the SHOOTER_MAP range, ramps to 0 outside the margin.
   *   <li>Lateral speed: score is 1 at rest, drops to 0 at {@link #lateralSpeedThresholdMPS}.
   * </ul>
   */
  public double getShotViabilityScale() {
    return getTurretAngleScore() * getDistanceScore() * getLateralSpeedScore();
  }

  private double getTurretAngleScore() {
    double absAngle = Math.abs(getIdealTurretAngle().in(Degrees));
    double hardLimit = TurretConstants.HARD_COUNTER_CLOCKWISE_LIMIT.in(Degrees);
    return MathUtil.clamp((hardLimit - absAngle) / turretApproachMarginDeg, 0.0, 1.0);
  }

  private double getDistanceScore() {
    double distance = getDistanceToTarget();
    if (distance >= shooterMapMinDistance && distance <= shooterMapMaxDistance) return 1.0;
    if (distance < shooterMapMinDistance)
      return MathUtil.clamp(
          (distance - (shooterMapMinDistance - distanceMarginM)) / distanceMarginM, 0.0, 1.0);
    return MathUtil.clamp(
        ((shooterMapMaxDistance + distanceMarginM) - distance) / distanceMarginM, 0.0, 1.0);
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
    return MathUtil.clamp(1.0 - lateral / lateralSpeedThresholdMPS, 0.0, 1.0);
  }

  // ── Data logging ──────────────────────────────────────────────────────────────

  /**
   * Logs a shot data point to the console for later extraction. Call this while the turret is aimed
   * and the shooter is running at a manually-set RPM.
   *
   * @param manualRPM the RPM value currently being tested
   */
  public void logDataPoint(double manualRPM) {
    double distance = getDistanceToTarget();
    System.out.println(
        String.format("SHOT_DATA | dist=%.2f | manualRPM=%.0f", distance, manualRPM));
    SmartDashboard.putString(
        "Tuning/LastLoggedPoint", String.format("%.2fm -> %.0f RPM", distance, manualRPM));
  }

  // ── Periodic ─────────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    latencyComp = SmartDashboard.getNumber("Tuning/LatencyComp", 0.15);
    // turretApproachMarginDeg = SmartDashboard.getNumber("Tuning/TurretApproachMarginDeg", 10.0);
    // distanceMarginM = SmartDashboard.getNumber("Tuning/DistanceMarginM", 0.5);
    // lateralSpeedThresholdMPS = SmartDashboard.getNumber("Tuning/LateralSpeedThresholdMPS", 1.5);

    SmartDashboard.putNumber("Ideal Turret Angle", getIdealTurretAngle().in(Degrees));
    SmartDashboard.putNumber("Distance To Target", getDistanceToTarget());
    // SmartDashboard.putNumber("Shot Viability", getShotViabilityScale());
    SmartDashboard.putBoolean("ShotCalc/isTargetingHub", isTargetingHub());
    SmartDashboard.putNumber("Tuning/MapSuggestedRPM", SHOOTER_MAP.get(getDistanceToTarget()).rpm);
  }
}
