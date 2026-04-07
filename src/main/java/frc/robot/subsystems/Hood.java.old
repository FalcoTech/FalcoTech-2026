// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final Servo turretHood = new Servo(0);

  // If Y-cable doesn't work with both servos, uncomment to drive them independently
  // private final Servo turretHood2 = new Servo(1);

  /** Creates a new Hood. */
  public Hood() {
    SmartDashboard.putNumber("Hood/Up Position", 0.5);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Current Position", turretHood.getPosition());
  }

  public Command hoodUp() {
    return this.runOnce(
        () -> {
          double pos = SmartDashboard.getNumber("Hood/Up Position", 0.5);
          turretHood.setPosition(pos);
          // turretHood2.setPosition(pos);
        });
  }

  public Command hoodDown() {
    return this.runOnce(
        () -> {
          turretHood.setPosition(0);
          // turretHood2.setPosition(0);
        });
  }

  public Command setHoodPosition(double position) {
    return this.runOnce(
        () -> {
          turretHood.setPosition(position);
          // turretHood2.setPosition(position);
        });
  }

  public void set(double position) {
    turretHood.setPosition(position);
    // turretHood2.setPosition(position);
  }
}
