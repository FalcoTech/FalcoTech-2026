// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Feeder extends SubsystemBase {
  private final SparkMax feedermotor = new SparkMax(30, MotorType.kBrushless);

  private SparkMaxConfig feedermotorconfig = new SparkMaxConfig();

  /** Creates a new Feeder. */
  public Feeder() {
    // feedermotorconfig.idleMode(IdleMode.kBrake);

    // feedermotor.configure(
        // feedermotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooterVoid(double speed) {
    feedermotor.set(speed);
  }

  public Command runFeeder(double speed) {
    return run(() -> feedermotor.set(speed));
  }

  public Command runFeeder(Supplier<Double> speedSupplier) {
    return run(() -> feedermotor.set(speedSupplier.get()));
  }

  public Command stopFeeder() {
    return run(() -> feedermotor.set(0));
  }
}
