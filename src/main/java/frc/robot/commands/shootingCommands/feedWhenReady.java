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
 *   <li><b>Hub shots:</b> all three gates must pass — turret within 3 degrees of setpoint, shooter
 *       within 150 RPM of setpoint, and robot speed below 0.15 m/s.
 *   <li><b>Non-hub shots (outpost/depot):</b> all gates are bypassed and the feeder runs freely.
 * </ul>
 */
public class feedWhenReady extends Command {

  // Tolerances — tune to match acceptable shot windows
  // TODO: Tune values for Autofeed
  private static final double ANGLE_TOLERANCE_DEG = 3.0;
  private static final double VELOCITY_TOLERANCE_RPM = 150.0;

  private static final double FEEDER_SPEED = 0.8; // TUNE
  // Robot must be below this speed (m/s) to fire — prevents shots while moving
  private static final double STOPPED_THRESHOLD_MPS = 0.15; // TUNE

  private final Feeder feeder;
  private final Turret turret;
  private final Shooter shooter;

  public feedWhenReady() {
    feeder = RobotContainer.feeder;
    turret = RobotContainer.turret;
    shooter = RobotContainer.shooter;
    // Only require feeder — turret/shooter are read-only here
    addRequirements(feeder);
  }

  private boolean isTurretReady(double toleranceMult) {
    return turret
        .getAngleSetpoint()
        .map(
            setpoint ->
                turret
                    .isNearAngle(setpoint, Degrees.of(ANGLE_TOLERANCE_DEG).times(toleranceMult))
                    .getAsBoolean())
        .orElse(false);
  }

  private boolean isShooterReady(double toleranceMult) {
    return shooter
        .getAngularVelocitySetpoint()
        .map(
            setpoint ->
                shooter
                    .isNearVelocity(setpoint, RPM.of(VELOCITY_TOLERANCE_RPM).times(toleranceMult))
                    .getAsBoolean())
        .orElse(false);
  }

  @Override
  public void execute() {
    boolean isTargetingHub = RobotContainer.shotCalculator.isTargetingHub();
    // For non-hub shots, we can be more lenient since they are less sensitive to aiming/speed
    // This allows the feeder to run sooner, which can help with cycle times
    boolean nonHubReady = isTurretReady(2.0) && isShooterReady(2.0);
    ChassisSpeeds speeds = RobotContainer.drivetrain.getState().Speeds;
    boolean robotStopped =
        // Convert to "total" speed and compare to threshold to determine if robot is effectively
        // stopped
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < STOPPED_THRESHOLD_MPS;

    // Non-hub shots : looser criteria so shooting happens sooner still matter to prevent
    // ineffective shots
    // Hub shots: require turret aimed, shooter up to speed, and robot stopped
    boolean shouldFeed =
        (!isTargetingHub && nonHubReady)
            || (isTurretReady(1.0) && isShooterReady(1.0) && robotStopped);

    SmartDashboard.putBoolean("FeedWhenReady/isTargetingHub", isTargetingHub);
    SmartDashboard.putBoolean("FeedWhenReady/robotStopped", robotStopped);
    SmartDashboard.putBoolean("FeedWhenReady/turretReady", isTurretReady(1.0));
    SmartDashboard.putBoolean("FeedWhenReady/shooterReady", isShooterReady(1.0));
    SmartDashboard.putBoolean(
        "FeedWhenReady/turretSetpointPresent", turret.getAngleSetpoint().isPresent());
    SmartDashboard.putBoolean(
        "FeedWhenReady/shooterSetpointPresent", shooter.getAngularVelocitySetpoint().isPresent());

    feeder.runFeederVoid(shouldFeed ? FEEDER_SPEED : 0.0);
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
