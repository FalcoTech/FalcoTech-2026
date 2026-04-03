// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Rotational turret subsystem driven by a TalonFX with closed-loop position control via YAMS {@link
 * yams.mechanisms.positional.Pivot}. Hard limits are +/-110 degrees; soft limits +/-100 degrees.
 * Zero degrees is defined as straight ahead on the robot.
 *
 * <p>A "Use Turret" SmartDashboard toggle allows the turret to be disabled on the fly for debugging
 * without redeploying.
 */
// TODO: Finish the YAMS rewrite by changing values

public class IndexShooter extends SubsystemBase {

  private final TalonFX indexshooterMotor = new TalonFX(CAN_IDs.indexshooterMotor);

  private final SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(1, 0, 0)
          .withFeedforward(new SimpleMotorFeedforward(.3, 0, 0.0))
          // Configure Motor and Mechanism properties
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
          .withIdleMode(MotorMode.COAST)
          .withMotorInverted(true)
          .withFollowers(Pair.of(indexshooterMotor, true))
          // Setup Telemetry
          .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
          // Power Optimization
          .withSupplyCurrentLimit(Amps.of(30))
          .withStatorCurrentLimit(Amps.of(20))
          .withVoltageCompensation(Volts.of(12));

  private final SmartMotorController motor =
      new TalonFXWrapper(indexshooterMotor, DCMotor.getKrakenX60(2), smcConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(2.5))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(5500))
          .withLowerSoftLimit(RPM.of(0))

          // Telemetry name and verbosity for the shooter.
          .withTelemetry("FlyWheelMech", TelemetryVerbosity.LOW);

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

  /* Creates new Shooter */
  public IndexShooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheel.updateTelemetry();
    SmartDashboard.putNumber("Flywheel Velocity", flywheel.getSpeed().in(RPM));
  }

  @Override
  public void simulationPeriodic() {
    flywheel.simIterate();
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return flywheel.set(dutyCycle);
  }

  /** Stops the flywheel (duty cycle 0). */
  public Command stop() {
    return flywheel.set(0);
  }

  /**
   * Runs the flywheel at a fixed angular velocity setpoint using closed-loop control.
   *
   * @param targetVelocity desired wheel speed (e.g. {@code RPM.of(4000)})
   */
  public Command setVelocity(AngularVelocity targetVelocity) {
    return flywheel.run(targetVelocity);
  }
}
