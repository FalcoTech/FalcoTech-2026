// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
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
  /** Creates a new ClimberElevator. */
  private SmartMotorControllerConfig climbElevConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(4, 0, 0, FeetPerSecond.of(0.5), FeetPerSecondPerSecond.of(1))
          .withFeedforward(new ElevatorFeedforward(0, 0, 0))
          .withTelemetry("ClimberElevMotor", TelemetryVerbosity.HIGH)
          .withGearing(
              new MechanismGearing(GearBox.fromStages("1:100"), Sprocket.fromStages("1:4")))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40));

  private SparkMax sparkClimbElev =
      new SparkMax(CAN_IDs.CLIMB_ELEVATOR_MOTOR, MotorType.kBrushless);

  private SmartMotorController climbElevMotorController =
      new SparkWrapper(sparkClimbElev, DCMotor.getNEO(1), climbElevConfig);

  private ElevatorConfig climbElevatorConfig =
      new ElevatorConfig(climbElevMotorController)
          .withStartingHeight(Inches.of(0.5))
          .withHardLimits(Inches.of(6), Inches.of(12))
          .withTelemetry("ClimbElevator", TelemetryVerbosity.HIGH)
          .withMass(Pounds.of(12));

  private Elevator climbElev = new Elevator(climbElevatorConfig);

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
}
