// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final Servo turretHood = new Servo(0);

  /** Creates a new Hood. */
  public Hood() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command hoodUp() {
    return Commands.runOnce(() -> turretHood.set(1));
  }

  public Command hoodDown() {
    return Commands.runOnce(() -> turretHood.set(0));
  }
}
