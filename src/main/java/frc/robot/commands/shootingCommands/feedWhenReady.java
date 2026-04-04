// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingCommands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Feeds game pieces into the shooter when readiness conditions are met. Only requires the {@link
 * Feeder} subsystem — turret and shooter are read-only.
 *
 * <p>Gating behavior depends on the current target:
 *
 * <ul>
 *   <li><b>Hub shots:</b> all three gates must pass — turret within tolerance of setpoint, shooter
 *       within tolerance of setpoint, and robot speed below threshold (unless ShootOnTheMove is
 *       enabled).
 *   <li><b>Non-hub shots (outpost/depot):</b> looser tolerances (2x) are applied.
 * </ul>
 */
public class feedWhenReady extends Command {

  // Tolerances — live-tunable via SmartDashboard under Tuning/ prefix
  private double angleToleanceDeg = 3.0;
  private double velocityToleranceRPM = 150.0;
  private double feederSpeed = 0.8;
  // Robot must be below this speed (m/s) to fire — prevents shots while moving
  private double stoppedThresholdMPS = 0.15;

  private final Feeder feeder;
  private final Turret turret;
  private final Shooter shooter;

  public feedWhenReady() {
    feeder = RobotContainer.feeder;
    turret = RobotContainer.turret;
    shooter = RobotContainer.shooter;
    // Only require feeder — turret/shooter are read-only here
    addRequirements(feeder);

    SmartDashboard.putNumber("Tuning/FeedAngleToleranceDeg", angleToleanceDeg);
    SmartDashboard.putNumber("Tuning/FeedVelocityToleranceRPM", velocityToleranceRPM);
    SmartDashboard.putNumber("Tuning/FeederSpeed", feederSpeed);
    SmartDashboard.putNumber("Tuning/StoppedThresholdMPS", stoppedThresholdMPS);
  }

  private boolean isTurretReady(double toleranceMult) {
    return turret
        .getAngleSetpoint()
        .map(
            setpoint ->
                turret
                    .isNearAngle(setpoint, Degrees.of(angleToleanceDeg).times(toleranceMult))
                    .getAsBoolean())
        .orElse(false);
  }

  private boolean isShooterReady(double toleranceMult) {
    return shooter
        .getAngularVelocitySetpoint()
        .map(
            setpoint ->
                shooter
                    .isNearVelocity(setpoint, RPM.of(velocityToleranceRPM).times(toleranceMult))
                    .getAsBoolean())
        .orElse(false);
  }

  @Override
  public void execute() {
    angleToleanceDeg = SmartDashboard.getNumber("Tuning/FeedAngleToleranceDeg", 3.0);
    velocityToleranceRPM = SmartDashboard.getNumber("Tuning/FeedVelocityToleranceRPM", 150.0);
    feederSpeed = SmartDashboard.getNumber("Tuning/FeederSpeed", 0.8);
    stoppedThresholdMPS = SmartDashboard.getNumber("Tuning/StoppedThresholdMPS", 0.15);
    boolean shootOnTheMove = SmartDashboard.getBoolean("Tuning/ShootOnTheMove", false);

    boolean isTargetingHub = RobotContainer.shotCalculator.isTargetingHub();
    // For non-hub shots, we can be more lenient since they are less sensitive to aiming/speed
    // This allows the feeder to run sooner, which can help with cycle times
    boolean nonHubReady = isTurretReady(2.0) && isShooterReady(2.0);
    ChassisSpeeds speeds = RobotContainer.drivetrain.getState().Speeds;
    boolean robotStopped =
        // Convert to "total" speed and compare to threshold to determine if robot is effectively
        // stopped
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < stoppedThresholdMPS;

    // Non-hub shots : looser criteria so shooting happens sooner still matter to prevent
    // ineffective shots
    // Hub shots: require turret aimed, shooter up to speed, and robot stopped (or ShootOnTheMove)
    boolean hubReady =
        isTurretReady(1.0) && isShooterReady(1.0) && (robotStopped || shootOnTheMove);
    boolean shouldFeed = (!isTargetingHub && nonHubReady) || hubReady;

    SmartDashboard.putBoolean("FeedWhenReady/isTargetingHub", isTargetingHub);
    SmartDashboard.putBoolean("FeedWhenReady/robotStopped", robotStopped);
    SmartDashboard.putBoolean("FeedWhenReady/turretReady", isTurretReady(1.0));
    SmartDashboard.putBoolean("FeedWhenReady/shooterReady", isShooterReady(1.0));
    SmartDashboard.putBoolean(
        "FeedWhenReady/turretSetpointPresent", turret.getAngleSetpoint().isPresent());
    SmartDashboard.putBoolean(
        "FeedWhenReady/shooterSetpointPresent", shooter.getAngularVelocitySetpoint().isPresent());

    feeder.runFeederVoid(shouldFeed ? feederSpeed : 0.0);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.runFeederVoid(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
