// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShotCalculator;
import java.util.function.Supplier;

public class ShooterSystem extends SubsystemBase {
  private final Launcher launcher = new Launcher();
  private final Turret turret = new Turret();

  private Supplier<AngularVelocity> launcherWheelVelocity = () -> DegreesPerSecond.of(0);

  private PIDController turretController = new PIDController(10, 0, 0);
  private PIDController launcherController = new PIDController(12, 0, 0);

  /* Creates a new ShooterSystem. */
  public ShooterSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Based on input from util function from ShotCalc that gives the pose for intended shot
    // calculate the necessary turret angle and launcher wheel velocity and set the motors to those
    // values

    Angle targetAngle = ShotCalculator.getCompensatedShotAngle();
    // Might need to put all the safety checks and hardstops/wrapping here but this is at least a
    // start
    if (SmartDashboard.getBoolean("autoAim", false) == true) {
      turret.setDefaultCommand(turret.setAngle(targetAngle));
    } else {
      turret.setDefaultCommand(turret.setDutyCycle(0));
      // Exmple code uses PID controllers here but I don't know why

    }
  }

  // Set the turret to a specific angle with our hardstop being at 0 degrees
  public Command aimAt(Angle turretAngle) {
    return turret.setAngle(turretAngle);
  }

  public Command aimAt(Supplier<Angle> turretAngle) {
    return turret.setAngle(turretAngle);
  }

  public Angle getTurretAngle() {
    return turret.getAngle();
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return launcher.set(dutyCycle);
  }

  public Command stop() {
    return launcher.set(0);
  }

  public AngularVelocity getVelocity() {
    return launcher.getVelocity();
  }

  public Command aimClockwise() {
    return turret.setDutyCycle(0.25);    
  }

  public Command aimCounterClockwise() {
    return turret.setDutyCycle(-0.25);
  }

  public Command aimStop() {
    return turret.setDutyCycle(0);
  }
}
