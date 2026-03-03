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

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Turret extends SubsystemBase {
  // private final SparkMax turretMotor =
  // new SparkMax(CAN_IDs.TURRET_MOTOR, SparkMax.MotorType.kBrushless);

  private final TalonFX turretMotor = new TalonFX(CAN_IDs.TURRET_MOTOR);
  // private TalonFXConfiguration WristMotorConfig = new TalonFXConfiguration();
  // private TalonFXConfigurator WristMotorConfigurator = turretMotor.getConfigurator();

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              35, 0, .75, DegreesPerSecond.of(1080), DegreesPerSecondPerSecond.of(2160))
          // .withLinearClosedLoopController(false)
          .withFeedforward(new SimpleMotorFeedforward(.3, 0, 0.0))
          // .withClosedLoopTolerance(Degrees.of(0.5)) //doesn't work with TalonFX
          // Configure Motor and Mechanism properties
          // .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 10)))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 10)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(true)
          //         // Setup Telemetry
          .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
          //         // Power Optimization
          .withStatorCurrentLimit(Amps.of(30));
  // .withClosedLoopRampRate(Seconds.of(0.25))
  // // .withOpenLoopRampRate(Seconds.of(0.25))
  // // .withVoltageCompensation(Volts.of(12)) // also doesn't work with TalonFX

  private final SmartMotorController turretSMC =
      //     // new SparkWrapper(turretMotor, DCMotor.getNEO(1), motorConfig);
      new TalonFXWrapper(turretMotor, DCMotor.getFalcon500(1), motorConfig);

  private final PivotConfig turretConfig =
      new PivotConfig(turretSMC)
          .withStartingPosition(Degrees.of(0))
          .withHardLimit(Degrees.of(-20), Degrees.of(220))
          .withSoftLimits(Degrees.of(-10), Degrees.of(210))
          .withTelemetry("TurretMech", TelemetryVerbosity.HIGH)
          .withMOI(Meters.of(0.25), Pounds.of(4));

  private final Pivot turret = new Pivot(turretConfig);

  /** Creates a new Turret. */
  public Turret() {}

  public Command setAngle(Angle targetAngle) {
    return turret.setAngle(targetAngle);
  }

  public void setAngleDirect(Angle targetAngle) {
    // turretSMC.setPosition(targetAngle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  // public Command sysId() {
  //   return turret.sysId(
  //       Volts.of(12), // Max voltage to apply during the test
  //       Volts.per(Second).of(0.5), // Step voltage per second
  //       Seconds.of(10) // Duration of the test
  //       );
  // }

  public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
    return turret.set(dutyCycleSupplier);
  }

  public Command setDutyCycle(double dutyCycle) {
    return turret.set(dutyCycle);
  }

  public Command stop() {
    return turret.set(0);
  }
  ;

  public void setDirectDutyCycle(double speed) {
    turretMotor.set(speed);
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
    turret.updateTelemetry();
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
