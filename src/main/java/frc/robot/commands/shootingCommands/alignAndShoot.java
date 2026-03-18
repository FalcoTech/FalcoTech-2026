// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shootingCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.ShotCalculator;

public class alignAndShoot extends ParallelCommandGroup {

  public alignAndShoot() {
    ShotCalculator shotCalculator = RobotContainer.shotCalculator;

    addCommands(
        RobotContainer.turret.setAngle(shotCalculator::getIdealTurretAngle),
        RobotContainer.shooter.setAngularVelocity(shotCalculator::getIdealShooterVelocity));

    addRequirements(shotCalculator);
  }
}
