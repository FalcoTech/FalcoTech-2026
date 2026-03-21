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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_IDs;
import java.util.Optional;
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
              35,
              0,
              .75,
              DegreesPerSecond.of(1080),
              DegreesPerSecondPerSecond.of(2160)) // Vel = 1080, Accel = 2160
          // .withLinearClosedLoopController(false)
          .withFeedforward(new SimpleMotorFeedforward(.3, 0, 0.0))
          // .withClosedLoopTolerance(Degrees.of(0.5)) //doesn't work with TalonFX
          // Configure Motor and Mechanism properties
          // .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 10)))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 10)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(true)
          //         // Setup Telemetry
          .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
          //         // Power Optimization
          .withStatorCurrentLimit(Amps.of(20));
  // .withClosedLoopRampRate(Seconds.of(0.25))
  // // .withOpenLoopRampRate(Seconds.of(0.25))
  // // .withVoltageCompensation(Volts.of(12)) // also doesn't work with TalonFX

  private final SmartMotorController turretSMC =
      //     // new SparkWrapper(turretMotor, DCMotor.getNEO(1), motorConfig);
      new TalonFXWrapper(turretMotor, DCMotor.getFalcon500(1), motorConfig);

  private final PivotConfig turretConfig =
      new PivotConfig(turretSMC)
          .withStartingPosition(Degrees.of(0))
          // Update to have 0 be forwards to reduce math overheard
          // .withStartingPosition(HARD_CLOCKWISE_LIMIT))?
          .withHardLimit(Degrees.of(-110), Degrees.of(110))
          .withSoftLimits(Degrees.of(-100), Degrees.of(100))
          .withTelemetry("TurretMech", TelemetryVerbosity.LOW)
          .withMOI(Meters.of(0.25), Pounds.of(4));

  private final Pivot turret = new Pivot(turretConfig);

  /** Creates a new Turret. */
  public Turret() {
    SmartDashboard.putBoolean("Use Turret", true);
  }

  public Command setAngle(Angle targetAngle) {
    return turret.setAngle(targetAngle);
  }

  public void setAngleDirect(Angle targetAngle) {
    turretSMC.setPosition(targetAngle);
  }

  public Command setAngle(Supplier<Angle> angleSupplier) {
    return turret.setAngle(angleSupplier);
  }

  public Angle getAngle() {
    return turret.getAngle();
  }

  public Trigger isNearAngle(Angle target, Angle tolerance) {
    return turret.isNear(target, tolerance);
  }

  public Optional<Angle> getAngleSetpoint() {
    return turret.getMechanismSetpoint();
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

  /**
   * Reset the encoder to the lowest position when the current threshhold is reached. Should be used
   * when the Mechanism position is unreliable, like startup. Threshhold is only detected if
   * exceeded for 0.2 seconds, and the motor moves less than 2 degrees per second.
   *
   * @param threshhold The current threshhold held when the Mechanism is at it's hard limit.
   * @return
   */
  public Command resetZeroToHardStop(Current threshold) {
    // copied/ based on the example from YAMS github
    Debouncer currentDebouncer = new Debouncer(0.2); // Current threshold must last 0.2s
    Voltage stopVoltage = Volts.of(0); // Voltage to set at end of Command
    Voltage runVoltage =
        Volts.of(-3); // Voltage to run mechanism with (variable upon mechanism), negative to run in
    // reverse
    Angle limitAngle =
        this.turretConfig
            .getLowerHardLimit()
            .get(); // Use Mechanism hard limit as the new start point
    AngularVelocity velocityThreshold =
        DegreesPerSecond.of(2); // Max amount of movement to be considered stopped

    return Commands.startRun(
            turretSMC::stopClosedLoopController, () -> turretSMC.setVoltage(runVoltage))
        .until(
            () ->
                currentDebouncer.calculate(
                    turretSMC.getStatorCurrent().gte(threshold)
                        && turretSMC.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              turretSMC.setVoltage(stopVoltage);
              turretSMC.setEncoderPosition(limitAngle);
              turretSMC.startClosedLoopController();
            });
  }

  public Command stop() {
    return turret.set(0);
  }

  public void setDirectDutyCycle(double speed) {
    turretMotor.set(speed);
  }

  @Override
  public void periodic() {
    turret.updateTelemetry();
    SmartDashboard.putNumber("Turret Position", getAngle().in(Degrees));

    if (!SmartDashboard.getBoolean("Use Turret", true)) {
      stop();
    }
    // SmartDashboard.putNumber("Turret Shot Angle", ShotCalculator.getIdealTurretAngle());
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
  }
}
