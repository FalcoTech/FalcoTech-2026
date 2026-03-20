// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingCommands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.Turret;

public class aimTurretAtTarget extends Command {

  private static final double SOFT_LIMIT_DEG =
      TurretConstants.HARD_COUNTER_CLOCKWISE_LIMIT.in(Degrees) - 10.0;

  private final Turret turret;
  private final ShotCalculator shotCalculator;

  public aimTurretAtTarget() {
    turret = RobotContainer.turret;
    shotCalculator = RobotContainer.shotCalculator;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.setAngleDirect(
        Degrees.of(
            MathUtil.clamp(
                shotCalculator.getIdealTurretAngle().in(Degrees),
                -SOFT_LIMIT_DEG,
                SOFT_LIMIT_DEG)));
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
