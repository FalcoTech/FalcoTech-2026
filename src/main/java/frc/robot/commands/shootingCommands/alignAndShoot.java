// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.util.ShotCalculator;

public class alignAndShoot extends ParallelCommandGroup {

  // Clamp setpoint inside soft limits so the PID never demands a position YAMS will refuse,
  // which causes the turret to thrash against the soft stop.
  private static final double SOFT_LIMIT_DEG =
      TurretConstants.HARD_COUNTER_CLOCKWISE_LIMIT.in(Degrees) - 10.0;

  public alignAndShoot() {
    ShotCalculator shotCalculator = RobotContainer.shotCalculator;

    addCommands(
        RobotContainer.turret.setAngle(
            () ->
                Degrees.of(
                    MathUtil.clamp(
                        shotCalculator.getIdealTurretAngle().in(Degrees),
                        -SOFT_LIMIT_DEG,
                        SOFT_LIMIT_DEG))),
        RobotContainer.shooter.setAngularVelocity(shotCalculator::getIdealShooterVelocity));

    addRequirements(shotCalculator);
  }
}
