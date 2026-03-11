// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

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
    if (VisionConstants.USE_LIMELIGHT) {
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // Feed Pigeon heading to both Limelights (required before MegaTag2 pose request)
      LimelightHelpers.SetRobotOrientation(
          VisionConstants.LIMELIGHT_MAIN, headingDeg, 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation(
          VisionConstants.LIMELIGHT_REAR, headingDeg, 0, 0, 0, 0, 0);

      // Only apply vision when not spinning fast
      if (Math.abs(omegaRps) < VisionConstants.VISION_OMEGA_CUTOFF_RPS) {
        var mainMeasurement =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_MAIN);
        if (mainMeasurement != null && mainMeasurement.tagCount > 0) {
          RobotContainer.drivetrain.addVisionMeasurement(
              mainMeasurement.pose,
              mainMeasurement.timestampSeconds,
              VecBuilder.fill(
                  VisionConstants.MEGATAG2_XY_STDDEV,
                  VisionConstants.MEGATAG2_XY_STDDEV,
                  VisionConstants.MEGATAG2_ROTATION_STDDEV));
        }

        var rearMeasurement =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_REAR);
        if (rearMeasurement != null && rearMeasurement.tagCount > 0) {
          RobotContainer.drivetrain.addVisionMeasurement(
              rearMeasurement.pose,
              rearMeasurement.timestampSeconds,
              VecBuilder.fill(
                  VisionConstants.MEGATAG2_XY_STDDEV,
                  VisionConstants.MEGATAG2_XY_STDDEV,
                  VisionConstants.MEGATAG2_ROTATION_STDDEV));
        }
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
    if (VisionConstants.USE_LIMELIGHT) {
      var llMeasurement =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LIMELIGHT_MAIN);
      if (llMeasurement != null && llMeasurement.tagCount >= 2) {
        RobotContainer.drivetrain.resetPose(llMeasurement.pose);
      }
    }
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
