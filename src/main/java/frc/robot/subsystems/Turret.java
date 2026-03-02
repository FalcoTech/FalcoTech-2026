// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
// TelemetryVerbosity import removed — telemetry calls commented out to avoid NT blocking
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {
  private final TalonFX turretMotor = new TalonFX(20);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              10, 0, 6, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1080))
          // .withClosedLoopController(
          // 10, 0, 0, DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1080))
          .withSoftLimit(Degrees.of(-20), Degrees.of(220))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 10))) // 5, 10
          // .withGearing(new MechanismGearing(GearBox.fromStages("1:1")))
          .withIdleMode(MotorMode.BRAKE)
          // .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new SimpleMotorFeedforward(0.01, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor =
      new TalonFXWrapper(turretMotor, DCMotor.getFalcon500(1), motorConfig);
  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  private final PivotConfig m_config =
      new PivotConfig(motor)
          .withHardLimit(Degrees.of(-20), Degrees.of(220))
          // .withTelemetry("TurretExample", TelemetryVerbosity.HIGH)
          .withStartingPosition(Degrees.of(0))
          .withMechanismPositionConfig(robotToMechanism)
          .withMOI(Meters.of(.3), Pounds.of(5));

  private final Pivot turret = new Pivot(m_config);

  /** Creates a new Turret. */
  public Turret() {}

  public Command setAngle(Angle targetAngle) {
    return turret.setAngle(targetAngle);
  }

  public void setAngleDirect(Angle targetAngle) {
    turret.setAngle(targetAngle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Command sysId() {
    return turret.sysId(
        Volts.of(12), // Max voltage to apply during the test
        Volts.per(Second).of(0.5), // Step voltage per second
        Seconds.of(10) // Duration of the test
        );
  }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command stop() {
    return turret.set(0);
  }

  @Override
  public void periodic() {
    // NOTE: previously this called `turret.updateTelemetry()` which triggers
    // YAMS/remote motor controller config refreshes. Those refreshes are
    // blocking and must not be invoked from the main scheduler loop.
    // Commenting it out prevents the "Do not apply or refresh configs
    // periodically, as configs are blocking" error observed on the Driver
    // Station. If you need telemetry, call updateTelemetry() once at init
    // or from a dedicated off-main-thread task.
  // telemetry refresh removed from periodic to avoid blocking NT/remote calls
  // turret.updateTelemetry();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
