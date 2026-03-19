// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private boolean useMegaTag2;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();

    // MegaTag2 with dual Limelights
    if (VisionConstants.USE_LIMELIGHT && !useMegaTag2) {
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // LimelightHelpers.SetRobotOrientation("", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement_main =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_MAIN);
      if (llMeasurement_main != null
          && llMeasurement_main.tagCount > 1
          && Math.abs(omegaRps) < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(
            llMeasurement_main.pose, llMeasurement_main.timestampSeconds);
      }
      var llMeasurement_rear =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_REAR);
      if (llMeasurement_rear != null
          && llMeasurement_rear.tagCount > 1
          && Math.abs(omegaRps) < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(
            llMeasurement_rear.pose, llMeasurement_rear.timestampSeconds);
      }
    }
    SmartDashboard.putNumber("Manual RPM Setpoint", RobotContainer.manualRPM);
    useMegaTag2 = SmartDashboard.getBoolean("Enable MegaTag2", useMegaTag2);
    // SmartDashboard.getBoolean("Enable MegaTag2", enableMegaTag2);

    if (useMegaTag2) {
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      var pitchDeg = RobotContainer.drivetrain.getPigeon2().getPitch();
      var rollDeg = RobotContainer.drivetrain.getPigeon2().getRoll();

      BaseStatusSignal.refreshAll(pitchDeg, rollDeg);

      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(
          VisionConstants.LIMELIGHT_MAIN,
          headingDeg,
          0,
          pitchDeg.getValueAsDouble(),
          0,
          rollDeg.getValueAsDouble(),
          0);
      LimelightHelpers.SetRobotOrientation(
          VisionConstants.LIMELIGHT_REAR,
          headingDeg,
          0,
          pitchDeg.getValueAsDouble(),
          0,
          rollDeg.getValueAsDouble(),
          0);
      var llMeasurement_main =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_MAIN);
      if (llMeasurement_main != null
          && llMeasurement_main.tagCount > 0
          && Math.abs(omegaRps) < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(
            llMeasurement_main.pose,
            llMeasurement_main.timestampSeconds,
            VecBuilder.fill(.5, .5, 9999999));
        // RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
        // llMeasurement.timestampSeconds);
      }
      var llMeasurement_rear =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_REAR);
      if (llMeasurement_rear != null
          && llMeasurement_rear.tagCount > 0
          && Math.abs(omegaRps) < 2.0) {
        RobotContainer.drivetrain.addVisionMeasurement(
            llMeasurement_rear.pose,
            llMeasurement_rear.timestampSeconds,
            VecBuilder.fill(.5, .5, 9999999));
        // RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
        // llMeasurement.timestampSeconds);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Use MegaTag1 to seed heading while robot is stationary pre-match.
    // MegaTag1 solves full pose (including rotation) from tags alone,
    // giving MegaTag2 an accurate heading baseline once the match starts.
    // if (VisionConstants.USE_LIMELIGHT) {
    //   var llMeasurement =
    //       LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_MAIN);
    //   SmartDashboard.putNumber("LL Tag Count", llMeasurement.tagCount);
    //   if (llMeasurement != null && llMeasurement.tagCount >= 1) {
    //     // RobotContainer.drivetrain.resetPose(llMeasurement.pose);
    //     RobotContainer.drivetrain.resetPose(llMeasurement.pose);
    //   }
    // }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
    // RobotContainer.leds.setAlliance(
    //     DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
    //         == DriverStation.Alliance.Red);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
