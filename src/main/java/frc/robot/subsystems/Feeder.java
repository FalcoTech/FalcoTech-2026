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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import java.util.function.Supplier;

public class Feeder extends SubsystemBase {
  private final SparkMax feederMotor = new SparkMax(CAN_IDs.FEEDER_MOTOR, MotorType.kBrushless);

  private SparkMaxConfig feederMotorconfig = new SparkMaxConfig();

  /** Creates a new Feeder. */
  public Feeder() {
    feederMotorconfig.idleMode(IdleMode.kBrake);

    feederMotor.configure(
        feederMotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Feeder Motor Output", feederMotor.getAppliedOutput());
  }

  public Command runFeeder(double speed) {
    return run(() -> feederMotor.set(speed));
  }

  public Command runFeeder(Supplier<Double> speedSupplier) {
    return run(() -> feederMotor.set(speedSupplier.get()));
  }

  public Command stopFeeder() {
    return run(() -> feederMotor.set(0));
  }

  public void runFeederVoid(double speed) {
    feederMotor.set(speed);
  }
}
