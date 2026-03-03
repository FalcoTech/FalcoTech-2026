// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
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

public class Intake extends SubsystemBase {

  private SmartMotorControllerConfig intakeArmMotorConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
          .withFeedforward(new ArmFeedforward(0, 0, 0))
          .withTelemetry("IntakeArmMotor", TelemetryVerbosity.HIGH)

          // Gearing to be set per Mike
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withMotorInverted(false)
          .withStatorCurrentLimit(Amps.of(40));

  private SparkMax intakeArmMotor = new SparkMax(CAN_IDs.INTAKE_ARM_MOTOR, MotorType.kBrushless);

  private SmartMotorController intakeArmMotorController =
      new SparkWrapper(intakeArmMotor, DCMotor.getNEO(1), intakeArmMotorConfig);

  private ArmConfig intakeArmConfig =
      new ArmConfig(intakeArmMotorController)
          // Soft limit is applied to the PID
          .withSoftLimits(Degrees.of(-5), Degrees.of(90))
          // Hard limits are applied to simulation
          .withHardLimit(Degrees.of(-10), Degrees.of(95))
          // Hopefully not issue again, is used to set encoder theoretically?
          .withStartingPosition(Degrees.of(80))
          .withLength(Inches.of(12))
          .withMass(Pounds.of(3))
          .withTelemetry("IntakeArm", TelemetryVerbosity.HIGH);

  private Arm intakeArm = new Arm(intakeArmConfig);


  // TODO: Have Logan add the code to add the intake Roller as another SparkMax

  /** Creates a new Intake. */
  public Intake() {}

  // Intake Arm will have 1 arm and also rollers

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeArm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intakeArm.simIterate();
  }

  /**
   * Set the angle of the arm, does not stop when the arm reaches the setpoint.
   *
   * @param angle Angle to go to.
   * @return A command.
   */
  public Command setAngle(Angle angle) {
    return intakeArm.run(angle);
  }

  /**
   * Set the angle of the arm, ends the command but does not stop the arm when the arm reaches the
   * setpoint.
   *
   * @param angle Angle to go to.
   * @return A Command
   */
  public Command setAngleAndStop(Angle angle, Angle tolerance) {
    return intakeArm.runTo(angle, tolerance);
  }

  /**
   * Set arm closed loop controller to go to the specified mechanism position.
   *
   * @param angle Angle to go to.
   */
  public void setAngleSetpoint(Angle angle) {
    intakeArm.setMechanismPositionSetpoint(angle);
  }

  /**
   * Move the arm up and down.
   *
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) {
    return intakeArm.set(dutycycle);
  }

  /** Run sysId on the {@link Arm} */
  public Command sysId() {
    return intakeArm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  public Command runIntake(Double speed)
    intakeRoller.set(speed);
}
