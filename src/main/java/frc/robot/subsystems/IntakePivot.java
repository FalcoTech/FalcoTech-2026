// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_IDs;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakePivot extends SubsystemBase {

  /** Creates a new IntakePivot. */
  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Mechanism Circumference is the distance traveled by each mechanism rotation converting
          // rotations to Inches.
          // .withMechanismCircumference(Inches.of(Inches.of(0.25).in(Inches) * 22))
          // Feedback Constants (PID Constants)
          .withClosedLoopController(4, 0, 0.2)
          // Feedforward Constants
          .withFeedforward(new ArmFeedforward(0.011, 0.8177, 0.01))
          // Telemetry name and verbosity level
          .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to
          // the
          // gearbox attached to your motor.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(9, 2)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(60));

  // Vendor motor controller object
  SparkMax spark = new SparkMax(CAN_IDs.INTAKEPIVOT_MOTOR, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  ArmConfig armConfig =
      new ArmConfig(sparkSmartMotorController)
          .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH) // Telemetry Name
          .withMass(Pounds.of(5)) // Mass of the Arm
          .withLength(Inches.of(14))
          .withStartingPosition(Degrees.of(110))
          .withHardLimit(Degrees.of(-5), Degrees.of(90)) // Hard limits defined
          .withSoftLimits(Degrees.of(0), Degrees.of(85)); // Limits imposed on the PID controller.

  private Arm intakePivot = new Arm(armConfig);

  public IntakePivot() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakePivot.updateTelemetry();
    SmartDashboard.putNumber("IntakePivot/Angle", intakePivot.getAngle().in(Degrees)); 
  }

  @Override
  public void simulationPeriodic() {
    intakePivot.simIterate();
  }

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   *
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) {
    return intakePivot.run(angle);
  }

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   *
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return intakePivot.run(angle);
  }

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the
   * setpoint.
   *
   * @param angle Angle to go to.
   * @param tolerance Angle tolerance for completion.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle, Angle tolerance) {
    return intakePivot.runTo(angle, tolerance);
  }

  public void setAngleSetpoint(Angle angle) {
    intakePivot.setMechanismPositionSetpoint(angle);
  }

  public Command runDutyCycle(Supplier<Double> dutyCycle) {
    return intakePivot.set(dutyCycle);
  }

  public Command runDutyCycle(double dutyCycle) {
    return intakePivot.set(dutyCycle);
  }

  public Trigger isInStoredPosition() {
    return intakePivot
        .isNear(Degrees.of(90), Degrees.of(5));
  }

  public Command stop() {
    return intakePivot.set(0);
  }
}
