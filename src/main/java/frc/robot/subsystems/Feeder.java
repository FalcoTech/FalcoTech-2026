// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import java.util.function.Supplier;

// Feeder will have 1 roller to feed the ball from hopper
public class Feeder extends SubsystemBase {
  private SparkMaxConfig feederConfig = new SparkMaxConfig();
  // I apparently can't recall how to do this

  //TODO: Finish configuring the feeder motor

  private final SparkMax feederMotor = new SparkMax(CAN_IDs.FEEDER_MOTOR, MotorType.kBrushless);

  /** Creates a new Feeder. */
  public Feeder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
