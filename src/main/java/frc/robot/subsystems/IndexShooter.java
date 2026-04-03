// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
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
public class IndexShooter extends SubsystemBase {
  // private final SparkMax indexshooterMotor =
  // new SparkMax(CAN_IDs.TURRET_MOTOR, SparkMax.MotorType.kBrushless);

  private final TalonFX indexshooterMotor = new TalonFX(CAN_IDs.indexshooterMotor);

  private final SmartMotorControllerConfig smcConfig =
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
  public IndexShooter() {
    // Idle mode is burned to flash via REV Hardware Client — verify it here at startup.
    // If this warning fires, reconnect the motor to the REV client and set coast mode, then burn.

    // if (sparkRight.configAccessor.getIdleMode() != IdleMode.kCoast) {

    System.err.println(
        "[Shooter] WARNING: Right flywheel follower idle mode is not Coast! Burn with REV Hardware Client.");
  }

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
}
