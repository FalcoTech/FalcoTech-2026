// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import java.util.function.Supplier;

/**
 * Dual -motor subsystem that spins game pieces through the hopper toward the feeder. Runs 2
 * brushless motor via TalonFX in coast mode with a 30 A current limit.
 */
public class SpinnerIndex extends SubsystemBase {
  private final TalonFX SpinnerIndexmotorright = new TalonFX(CAN_IDs.SPINNERINDEXRIGHT_MOTOR);

  private final TalonFX SpinnerIndexmotorleft = new TalonFX(CAN_IDs.SPINNERINDEXLEFT_MOTOR);

  /** Creates a new HopperPush. */
  public SpinnerIndex() {
    var talonFXConfiguratorright = SpinnerIndexmotorright.getConfigurator();
    var motorConfigsright = new MotorOutputConfigs();
    motorConfigsright.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfiguratorright.apply(motorConfigsright);

    var talonFXConfiguratorleft = SpinnerIndexmotorright.getConfigurator();
    var motorConfigsleft = new MotorOutputConfigs();
    motorConfigsleft.Inverted = InvertedValue.Clockwise_Positive;

    SpinnerIndexmotorleft.setControl(
        new Follower(CAN_IDs.SPINNERINDEXRIGHT_MOTOR, MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runSpinnerIndexVoid(double speed) {
    SpinnerIndexmotorright.set(speed);
  }

  public Command runSpinnerIndex(double speed) {
    return run(() -> SpinnerIndexmotorright.set(speed));
  }

  public Command runSpinnerIndex(Supplier<Double> speedSupplier) {
    return run(() -> SpinnerIndexmotorright.set(speedSupplier.get()));
  }

  public Command stopSpinnerIndex() {
    return run(() -> SpinnerIndexmotorright.set(0));
  }
}
