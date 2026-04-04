// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.CurrentLimits;
import java.util.function.Supplier;

/**
 * Dual-motor subsystem that spins game pieces through the hopper toward the feeder. Runs 2
 * brushless motors via TalonFX in coast mode with current limits and VoltageOut control for
 * consistent output regardless of battery voltage.
 */
public class SpinnerIndex extends SubsystemBase {
  private static final double NOMINAL_VOLTAGE = 12.0;

  private final TalonFX SpinnerIndexmotorright = new TalonFX(CAN_IDs.SPINNERINDEXRIGHT_MOTOR);

  private final TalonFX SpinnerIndexmotorleft = new TalonFX(CAN_IDs.SPINNERINDEXLEFT_MOTOR);

  private final VoltageOut voltageRequest = new VoltageOut(0);

  /** Creates a new SpinnerIndex. */
  public SpinnerIndex() {
    var talonFXConfiguratorright = SpinnerIndexmotorright.getConfigurator();
    var motorConfigsright = new MotorOutputConfigs();
    motorConfigsright.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfiguratorright.apply(motorConfigsright);
    talonFXConfiguratorright.apply(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimits.SPINNER_INDEX_STATOR)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimits.SPINNER_INDEX_SUPPLY)
            .withSupplyCurrentLimitEnable(true));

    var talonFXConfiguratorleft = SpinnerIndexmotorleft.getConfigurator();
    var motorConfigsleft = new MotorOutputConfigs();
    motorConfigsleft.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfiguratorleft.apply(motorConfigsleft);
    talonFXConfiguratorleft.apply(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(CurrentLimits.SPINNER_INDEX_STATOR)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(CurrentLimits.SPINNER_INDEX_SUPPLY)
            .withSupplyCurrentLimitEnable(true));

    SpinnerIndexmotorleft.setControl(
        new Follower(CAN_IDs.SPINNERINDEXRIGHT_MOTOR, MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runSpinnerIndexVoid(double speed) {
    SpinnerIndexmotorright.setControl(voltageRequest.withOutput(speed * NOMINAL_VOLTAGE));
  }

  public Command runSpinnerIndex(double speed) {
    return run(
        () ->
            SpinnerIndexmotorright.setControl(voltageRequest.withOutput(speed * NOMINAL_VOLTAGE)));
  }

  public Command runSpinnerIndex(Supplier<Double> speedSupplier) {
    return run(
        () ->
            SpinnerIndexmotorright.setControl(
                voltageRequest.withOutput(speedSupplier.get() * NOMINAL_VOLTAGE)));
  }

  public Command stopSpinnerIndex() {
    return run(() -> SpinnerIndexmotorright.setControl(voltageRequest.withOutput(0)));
  }
}
