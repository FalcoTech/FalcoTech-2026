// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CAN_IDs;
// import java.util.function.Supplier;

// /**
//  * Single-motor subsystem that pushes game pieces through the hopper toward the feeder. Runs a
//  * brushless motor via SparkMax in coast mode with a 30 A current limit.
//  */
// public class HopperPush extends SubsystemBase {
//   private final SparkMax HopperPushmotor =
//       new SparkMax(CAN_IDs.HOPPERPUSH_MOTOR, MotorType.kBrushless);

//   private SparkMaxConfig HopperPushmotorconfig = new SparkMaxConfig();

//   /** Creates a new HopperPush. */
//   public HopperPush() {
//     HopperPushmotorconfig.idleMode(IdleMode.kCoast);
//     HopperPushmotorconfig.voltageCompensation(11);
//     HopperPushmotorconfig.smartCurrentLimit(30);

//     HopperPushmotor.configure(
//         HopperPushmotorconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public void runHopperPushVoid(double speed) {
//     HopperPushmotor.set(speed);
//   }

//   public Command runHopperPush(double speed) {
//     return run(() -> HopperPushmotor.set(speed));
//   }

//   public Command runHopperPush(Supplier<Double> speedSupplier) {
//     return run(() -> HopperPushmotor.set(speedSupplier.get()));
//   }

//   public Command stopHopperPush() {
//     return run(() -> HopperPushmotor.set(0));
//   }
// }
