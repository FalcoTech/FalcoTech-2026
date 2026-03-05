// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.ClimbConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimberPivot extends SubsystemBase {

  private SmartMotorControllerConfig climbArmControllerConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(
              ClimbConstants.ARM_PIVOT_kP,
              ClimbConstants.ARM_PIVOT_kI,
              ClimbConstants.ARM_PIVOT_kD,
              ClimbConstants.ARM_PIVOT_MAX_VELOCITY,
              ClimbConstants.ARM_PIVOT_MAX_ACCELERATION)
          .withFeedforward(
              new ArmFeedforward(
                  ClimbConstants.ArmPivot_FF_kS,
                  ClimbConstants.ArmPivot_FF_kG,
                  ClimbConstants.ArmPivot_FF_kV))
          .withTelemetry("ClimberArmMotor", TelemetryVerbosity.HIGH)
          .withGearing(
              new MechanismGearing(
                  GearBox.fromStages(ClimbConstants.Arm_GEARBOX_STAGES),
                  Sprocket.fromStages(ClimbConstants.Arm_SPROCKET_STAGES)))
          .withMotorInverted(false)
          .withIdleMode(MotorMode.BRAKE)
          .withStatorCurrentLimit(ClimbConstants.ARM_STATOR_CURRENT_LIMIT);

  private SparkMax sparkClimbArm = new SparkMax(CAN_IDs.CLIMB_Arm_MOTOR, MotorType.kBrushless);

  private SmartMotorController climbArmMotorController =
      new SparkWrapper(sparkClimbArm, DCMotor.getNEO(1), climbArmControllerConfig);

  private ArmConfig climbArmConfig =
      new ArmConfig(climbArmMotorController)
          // .withStartingPosition(Degrees.of(ClimbConstants.Arm_STARTING_HEIGHT))
          .withHardLimit(ClimbConstants.Arm_MIN_Position, ClimbConstants.Arm_MAX_Position)
          .withTelemetry("ClimbArm", TelemetryVerbosity.HIGH)
          .withMass(ClimbConstants.Arm_MASS);

  private Arm climbArm = new Arm(climbArmConfig);

  public ClimberPivot() {}

  // Climber has one or 2 elevator motors and 1 pivot motor

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climbArm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    climbArm.simIterate();
  }
}
