// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_IDs;
import frc.robot.util.ShotCalculator;

import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter extends SubsystemBase {

  private final SparkMax sparkLeft =
      new SparkMax(CAN_IDs.FLYWHEEL_MOTOR_LEFT, MotorType.kBrushless);
  private final SparkMax sparkRight =
      new SparkMax(CAN_IDs.FLYWHEEL_MOTOR_RIGHT, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          //         // Feedback Constants (PID Constants)
          .withClosedLoopController(0.0125, 0, 0, RPM.of(5000), DegreesPerSecondPerSecond.of(10000))
          .withSimClosedLoopController(.2, 0, 0, RPM.of(5700), DegreesPerSecondPerSecond.of(10000))
          .withFeedforward(new SimpleMotorFeedforward(0.124, .124, 0))
          .withSimFeedforward(new SimpleMotorFeedforward(0, .124, 0))
          //         // Telemetry name and verbosity level

          .withGearing(new MechanismGearing(GearBox.fromStages("1:1")))

          //         // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withTelemetry("FlyWheelMotors", TelemetryVerbosity.HIGH)
          .withVoltageCompensation(Volts.of(12))
          .withFollowers(Pair.of(sparkRight, true));

  private final SmartMotorController motor =
      new SparkWrapper(sparkLeft, DCMotor.getNEO(2), smcConfig);

  private final FlyWheelConfig flywheelConfig =
      new FlyWheelConfig(motor)
          // Diameter of the flywheel.
          .withDiameter(Inches.of(4))
          // Mass of the flywheel.
          .withMass(Pounds.of(2.5))
          // Maximum speed of the shooter.
          .withUpperSoftLimit(RPM.of(2000))
          .withLowerSoftLimit(RPM.of(0))

          // Telemetry name and verbosity for the shooter.
          .withTelemetry("FlyWheelMech", TelemetryVerbosity.HIGH);

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

  /* Creates new Shooter */
  public Shooter() {
    // Idle mode is burned to flash via REV Hardware Client — verify it here at startup.
    // If this warning fires, reconnect the motor to the REV client and set coast mode, then burn.
    if (sparkRight.configAccessor.getIdleMode() != IdleMode.kCoast) {
      System.err.println(
          "[Shooter] WARNING: Right flywheel follower idle mode is not Coast! Burn with REV Hardware Client.");
    }
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

  public Command stop() {
    return flywheel.set(0);
  }

  public Command setVelocity(AngularVelocity targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  public Command setAngularVelocity(Supplier<AngularVelocity> targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  public Command setVelocity(LinearVelocity targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  public Command setLinearVelocity(Supplier<LinearVelocity> targetVelocity) {
    return flywheel.run(targetVelocity);
  }

  public AngularVelocity getVelocity() {
    return flywheel.getSpeed();
  }

  public Trigger isNearVelocity(AngularVelocity target, AngularVelocity tolerance) {
    return flywheel.isNear(target, tolerance);
  }

  public Command sysId() {
    return flywheel.sysId(
        Volts.of(12), // Max voltage to apply during the test
        Volts.per(Second).of(0.5), // Step voltage per second
        Seconds.of(30) // Duration of the test
        );
  }

  public Command runVelocityStepTest() {
    final double STEP_DURATION = 5;
    return flywheel
        .run(RPM.of(1000))
        .withTimeout(STEP_DURATION)
        .andThen(flywheel.run(RPM.of(2000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(3000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(4000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(3000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(2000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(1000)).withTimeout(STEP_DURATION))
        .andThen(flywheel.run(RPM.of(0)));
  }

  // public Command setShooterToTargetSpeed(){
    // return setAngularVelocity(RPM.of(ShotCalculator))
  // }
}
