// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import javax.print.attribute.standard.RequestingUserName;

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
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSlide extends SubsystemBase {

  /** Creates a new IntakeSlide. */
  

  private SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Mechanism Circumference is the distance traveled by each mechanism rotation converting
          // rotations to Inches.
          .withMechanismCircumference(Inches.of(Inches.of(0.25).in(Inches) * 22))
          // Feedback Constants (PID Constants)
          .withClosedLoopController(
              25, 0, 0, InchesPerSecond.of(2), InchesPerSecondPerSecond.of(2))
          .withSimClosedLoopController(
              25, 0, 0, InchesPerSecond.of(2), InchesPerSecondPerSecond.of(2))
          // Feedforward Constants
          .withFeedforward(new ElevatorFeedforward(.1, 0, 0))
          .withSimFeedforward(new ElevatorFeedforward(.1, 0, 0))
          // Telemetry name and verbosity level
          .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
          // Gearing from the motor rotor to final shaft.
          // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to
          // the
          // gearbox attached to your motor.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 100)))
          // Motor properties to prevent over currenting.
          .withMotorInverted(true)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40));

  // Vendor motor controller object
  SparkMax spark = new SparkMax(CAN_IDs.INTAKESLIDE_MOTOR, MotorType.kBrushless);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController =
      new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  ElevatorConfig elevconfig =
      new ElevatorConfig(sparkSmartMotorController)
          .withStartingHeight(Inches.of(0.5)) // Starting height of the IntakeSlide
          .withTelemetry("IntakeSlide", TelemetryVerbosity.HIGH) // Telemetry Name
          .withMass(Pounds.of(1)) // Mass of the carraige
          .withStartingHeight(Inches.of(0.5)) // Starting height of the IntakeSlide
        //   .Wi(Pounds.of(1)) // Weight of the carriage.
          .withAngle(Degrees.of(0)) // Parallel to the ground, linear slide.
          .withHardLimits(Inches.of(-100), Inches.of(120)) // Hard limits defined
          .withSoftLimits(Inches.of(-80.25), Inches.of(100)); // Limits imposed on the PID controller.

   

          private Elevator intakeSlide = new Elevator(elevconfig);

    public IntakeSlide() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  intakeSlide.updateTelemetry();
}

@Override
public void simulationPeriodic()  {
    intakeSlide.simIterate();
}

public Command setHeight(Distance Height) {
    return intakeSlide.run(Height); 
}
public Command setHeightAndStop(Distance height, Distance tolerance) {
    return intakeSlide.runTo(height, tolerance);
}

public Command runDutyCycle(Supplier<Double> dutyCycle){
    return intakeSlide.set(dutyCycle);
}
public Command runDutyCycle(double dutyCycle){
    return intakeSlide.set(dutyCycle);
}
}