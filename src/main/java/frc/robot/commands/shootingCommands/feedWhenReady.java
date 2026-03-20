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

public class feedWhenReady extends Command {

  // Tolerances — tune to match acceptable shot windows
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
    // and are already required by the concurrent alignAndShoot command
    addRequirements(feeder);
  }

  @Override
  public void execute() {
    boolean turretReady =
        turret
            .getAngleSetpoint()
            .map(
                setpoint ->
                    turret.isNearAngle(setpoint, Degrees.of(ANGLE_TOLERANCE_DEG)).getAsBoolean())
            .orElse(false);
    boolean shooterReady =
        shooter
            .getAngularVelocitySetpoint()
            .map(
                setpoint ->
                    shooter.isNearVelocity(setpoint, RPM.of(VELOCITY_TOLERANCE_RPM)).getAsBoolean())
            .orElse(false);

    // Speed gate — robot must be stopped to fire
    ChassisSpeeds speeds = RobotContainer.drivetrain.getState().Speeds;
    boolean robotStopped =
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < STOPPED_THRESHOLD_MPS;

    // Hub shots require full readiness gate; close-range targets (outpost/depot) feed freely
    boolean isTargetingHub = RobotContainer.shotCalculator.isTargetingHub();
    boolean mechanismsReady = !isTargetingHub || (turretReady && shooterReady);

    SmartDashboard.putBoolean("FeedWhenReady/turretReady", turretReady);
    SmartDashboard.putBoolean("FeedWhenReady/shooterReady", shooterReady);
    SmartDashboard.putBoolean("FeedWhenReady/isTargetingHub", isTargetingHub);
    SmartDashboard.putBoolean("FeedWhenReady/robotStopped", robotStopped);
    SmartDashboard.putBoolean("FeedWhenReady/turretSetpointPresent",
        turret.getAngleSetpoint().isPresent());
    SmartDashboard.putBoolean("FeedWhenReady/shooterSetpointPresent",
        shooter.getAngularVelocitySetpoint().isPresent());

    feeder.runFeederVoid(mechanismsReady && robotStopped ? FEEDER_SPEED : 0.0);
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
