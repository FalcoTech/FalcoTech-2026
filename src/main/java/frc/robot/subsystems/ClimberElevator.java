// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.ClimbConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimberElevator extends SubsystemBase {
  private Distance pitch = Inches.of(0.25);
  private int teeth = 25;
  private SparkMax sparkClimbElev =
      new SparkMax(CAN_IDs.CLIMB_ELEVATOR_MOTOR, MotorType.kBrushless);

  private SmartMotorControllerConfig climbElevConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(pitch.times(teeth))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              ClimbConstants.ELEVATOR_kP,
              ClimbConstants.ELEVATOR_kI,
              ClimbConstants.ELEVATOR_kD,
              ClimbConstants.ELEVATOR_MAX_VELOCITY,
              ClimbConstants.ELEVATOR_MAX_ACCELERATION)
          .withFeedforward(
              new ElevatorFeedforward(
                  ClimbConstants.ELEVATOR_FF_kS,
                  ClimbConstants.ELEVATOR_FF_kG,
                  ClimbConstants.ELEVATOR_FF_kV))
          .withSoftLimit(
              ClimbConstants.ELEVATOR_MIN_HEIGHT,
              ClimbConstants.ELEVATOR_MAX_HEIGHT.minus(Inches.of(0.125)))
          .withTelemetry("ClimberElevMotor", TelemetryVerbosity.HIGH)
          .withGearing(
              new MechanismGearing(
                  GearBox.fromStages(ClimbConstants.ELEVATOR_GEARBOX_STAGES),
                  Sprocket.fromStages(ClimbConstants.ELEVATOR_SPROCKET_STAGES)))
          // .withWheelDiameter(Inches.of(2))
          // .withExternalEncoder(sparkClimbElev.getAbsoluteEncoder())
          .withVoltageCompensation(Constants.VOLTAGE_COMP)
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(ClimbConstants.ELEVATOR_STATOR_CURRENT_LIMIT);

  private SmartMotorController climbElevMotorController =
      new SparkWrapper(sparkClimbElev, DCMotor.getNEO(1), climbElevConfig);

  private ElevatorConfig climbElevatorConfig =
      new ElevatorConfig(climbElevMotorController)
          .withStartingHeight(
              ClimbConstants
                  .ELEVATOR_STARTING_HEIGHT) // The starting position should ONLY be defined if you
          // are NOT using an absolute encoder.
          .withHardLimits(ClimbConstants.ELEVATOR_MIN_HEIGHT, ClimbConstants.ELEVATOR_MAX_HEIGHT)
          .withTelemetry("ClimbElevator", TelemetryVerbosity.HIGH)
          .withMass(ClimbConstants.ELEVATOR_MASS);

  private Elevator climbElev = new Elevator(climbElevatorConfig);

  /** Creates a new ClimberElevator. */
  public ClimberElevator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbElev.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    climbElev.simIterate();
  }

  /**
   * Set the height of the elevator and does not end the command when reached.
   *
   * @param angle Distance to go to.
   * @return a Command
   */
  public Command setHeight(Distance height) {
    return climbElev.run(height);
  }

  /**
   * Set the height of the elevator and ends the command when reached, but not the closed loop
   * controller.
   *
   * @param angle Distance to go to.
   * @return A Command
   */
  public Command setHeightAndStop(Distance height, Distance tolerance) {
    return climbElev.runTo(height, tolerance);
  }

  /**
   * Set the elevators closed loop controller setpoint.
   *
   * @param angle Distance to go to.
   */
  public void setHeightSetpoint(Distance height) {
    climbElev.setMeasurementPositionSetpoint(height);
  }

  /**
   * Move the elevator up and down.
   *
   * @param dutycycle [-1, 1] speed to set the elevator too.
   */
  public Command set(double dutycycle) {
    return climbElev.set(dutycycle);
  }

  // TODO: Add command to handle a switch to duty cycle once motor is loaded with robot weight
}
