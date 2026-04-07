// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.CurrentLimits;
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
          .withStatorCurrentLimit(CurrentLimits.INTAKE_PIVOT_STATOR)
          .withVoltageCompensation(Volts.of(12));

  // Vendor motor controller object
  SparkMax spark = new SparkMax(CAN_IDs.INTAKEPIVOT_MOTOR, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController intakeArmSMC = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  ArmConfig armConfig =
      new ArmConfig(intakeArmSMC)
          .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH) // Telemetry Name
          .withMass(Pounds.of(5)) // Mass of the Arm
          .withLength(Inches.of(14))
          .withStartingPosition(Degrees.of(110))
          .withHardLimit(Degrees.of(-5), Degrees.of(90)) // Hard limits defined
          .withSoftLimits(Degrees.of(0), Degrees.of(85)); // Limits imposed on the PID controller.

  private Arm intakePivot = new Arm(armConfig);

  public IntakePivot() {
    SmartDashboard.putData(this);
  }

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

  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   *
   * @param angle Angle to go to.
   */
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
    return intakePivot.isNear(Degrees.of(90), Degrees.of(5));
  }

  public Command stop() {
    return intakePivot.set(0);
  }

  /**
   * Reset the encoder to the max position when the current threshhold is reached. Should be used
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
        Volts.of(8); // Voltage to run mechanism with (variable upon mechanism), negative to run in
    // reverse
    // TODO: I think this is still goofy and not working right
    Angle limitAngle =
        this.armConfig.getUpperHardLimit().get(); // Use Mechanism hard limit as the new start point
    AngularVelocity velocityThreshold =
        DegreesPerSecond.of(2); // Max amount of movement to be considered stopped

    return Commands.startRun(
            intakeArmSMC::stopClosedLoopController, () -> intakeArmSMC.setVoltage(runVoltage))
        .until(
            () ->
                currentDebouncer.calculate(
                    intakeArmSMC.getStatorCurrent().gte(threshold)
                        && intakeArmSMC.getMechanismVelocity().abs(DegreesPerSecond)
                            <= velocityThreshold.in(DegreesPerSecond)))
        .finallyDo(
            () -> {
              intakeArmSMC.setVoltage(stopVoltage);
              intakeArmSMC.setEncoderPosition(limitAngle);
              intakeArmSMC.startClosedLoopController();
            });
  }

  public Command resetEncoderToLimit(){
     return runOnce(() -> intakeArmSMC.setEncoderPosition(Degrees.of(110)));
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {
  //     builder.addStringProperty(
  //         "Command",
  //         () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null",
  //         null);
  
  //     // builder.addDoubleProperty("Current Position", () -> intakePivot.getAngle().in(Degrees), null);
  //     builder.addDoubleProperty("Target Position", () -> intakePivot.getMechanismSetpoint().orElse(Degrees.of(0)).in(Degrees), value -> intakePivot.setAngle(Degrees.of(value)));
  // }
}
