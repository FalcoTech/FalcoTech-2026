// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import java.util.function.Supplier;

public class IntakeRoller extends SubsystemBase {
  // private final SparkMax intakeRollerMotor = new SparkMax(CAN_IDs.INTAKEROLLER_MOTOR,
  // MotorType.kBrushless);
  private final SparkFlex intakeRollerMotor =
      new SparkFlex(CAN_IDs.INTAKEROLLER_MOTOR, MotorType.kBrushless);

  private SparkFlexConfig intakerollerMotorconfig = new SparkFlexConfig();

  // /** Creates a new Feeder. */
  public IntakeRoller() {
    intakerollerMotorconfig.idleMode(IdleMode.kBrake);
    intakerollerMotorconfig.inverted(true);

    intakeRollerMotor.configure(
        intakerollerMotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntakeRollers(double speed) {
    return run(() -> intakeRollerMotor.set(speed));
  }

  public Command runIntakeRollers(Supplier<Double> speedSupplier) {
    return run(() -> intakeRollerMotor.set(speedSupplier.get()));
  }

  public Command stopIntakeRollers() {
    return run(() -> intakeRollerMotor.set(0));
  }
}
