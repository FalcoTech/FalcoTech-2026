// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Elevator extends SubsystemBase {

//   /** Creates a new Elevator. */
//   public Elevator() {}

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

// private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
//   .withControlMode(ControlMode.CLOSED_LOOP)
//   // Mechanism Circumference is the distance traveled by each mechanism rotation converting
// rotations to meters.
//   .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
//   // Feedback Constants (PID Constants)
//   .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
//   .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5),
// MetersPerSecondPerSecond.of(0.5))
//   // Feedforward Constants
//   .withFeedforward(new ElevatorFeedforward(0, 0, 0))
//   .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
//   // Telemetry name and verbosity level
//   .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//   // Gearing from the motor rotor to final shaft.
//   // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the
// gearbox attached to your motor.
//   .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//   // Motor properties to prevent over currenting.
//   .withMotorInverted(false)
//   .withIdleMode(MotorMode.BRAKE)
//   .withStatorCurrentLimit(Amps.of(40))
//   .withClosedLoopRampRate(Seconds.of(0.25))
//   .withOpenLoopRampRate(Seconds.of(0.25))

// // Vendor motor controller object
// SparkMax spark = new SparkMax(4, MotorType.kBrushless);

// // Create our SmartMotorController from our Spark and config with the NEO.
// private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark,
// DCMotor.getNEO(1), smcConfig)
//      ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
//       .withStartingHeight(Meters.of(0.5)) // Starting height of the Elevator
//       .withHardLimits(Meters.of(0), Meters.of(3)) // Hard limits defined
//       .withTelemetry("Elevator", TelemetryVerbosity.HIGH) // Telemetry Name
//       .withMass(Pounds.of(16)); // Mass of the carraige

//     private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
//       .withStartingHeight(Meters.of(0.5)); // Starting height of the Elevator

//     private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
//       .withMass(Pounds.of(16)); // Weight of the carriage.

//     private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
//       .withAngle(Degrees.of(0)); // Parallel to the ground, linear slide.

//     private ElevatorConfig elevconfig = new ElevatorConfig(sparkSmartMotorController)
//       .withHardLimits(Meters.of(0), Meters.of(3)) // Hard limits defined
//       .withSoftLimits(Meters.of(0), Meters.of(2.5)); // Limits imposed on the PID controller.

// }
