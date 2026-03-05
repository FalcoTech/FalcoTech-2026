// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PathPlanningConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  // Drive speeds
  private static double MaxSpeed =
      .5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private static double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * ControllerConstants.DEADBAND)
          .withRotationalDeadband(
              MaxAngularRate * ControllerConstants.DEADBAND) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.Velocity); // Use open-loop control for drive motors

  public static final SwerveRequest.RobotCentric driveRobotCentric =
      new SwerveRequest.RobotCentric()
          .withDeadband(MaxSpeed * ControllerConstants.DEADBAND)
          .withRotationalDeadband(MaxAngularRate * ControllerConstants.DEADBAND)
          .withDriveRequestType(DriveRequestType.Velocity);

  private final SwerveRequest.SwerveDriveBrake XForm = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public static final PathConstraints pathFindConstraints =
      new PathConstraints(
          PathPlanningConstants.MAX_PATH_SPEED,
          PathPlanningConstants.MAX_PATH_ACCELERATION,
          PathPlanningConstants.MAX_ANGULAR_SPEED * (Math.PI / 180),
          PathPlanningConstants.MAX_ANGULAR_ACCELERATION);

  private final SendableChooser<Command> autoChooser;

  // Joysticks
  private final CommandXboxController Pilot = new CommandXboxController(0);
  private final CommandXboxController Copilot = new CommandXboxController(1);

  // Subsystems
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public static final IntakeArm intake = new IntakeArm();
  public static final Feeder feeder = new Feeder();
  public static final Hopper hopper = new Hopper();
  public static final LEDS leds = new LEDS();
  public static final Turret turret = new Turret();
  public static final Shooter shooter = new Shooter();
  public static final ClimberElevator climbElevator = new ClimberElevator();

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    RegisterNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("Tests");
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();

    SmartDashboard.putBoolean("Enable MegaTag2", false);

    // Push the Git Commit and Branch to SmartDashbaord
    SmartDashboard.putString(
        "Git Info", BuildConstants.BUILD_DATE.concat(" on ").concat(BuildConstants.GIT_BRANCH));
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -Pilot.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -Pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -Pilot.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    Pilot.a().whileTrue(drivetrain.applyRequest(() -> XForm));
    // Pilot.b().whileTrue(drivetrain.applyRequest(() ->
    //     point.withModuleDirection(new Rotation2d(-Pilot.getLeftY(), -Pilot.getLeftX()))
    // ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    Pilot.back().and(Pilot.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    Pilot.back().and(Pilot.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    Pilot.start().and(Pilot.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    Pilot.start().and(Pilot.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Zero the gyro on the robot.
    Pilot.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    // shooter.setDefaultCommand(shooter.stop());
    // Copilot.a().whileTrue(shooter.set(0.5));
    // // Copilot.b().whileTrue(shooter.set(0.25));
    // // Copilot.y().whileTrue(shooter.set(1));
    // Copilot.y().whileTrue(shooter.setVelocity(RPM.of(1000)));

    // Copilot.x().whileTrue(shooter.stop());

    // turret.setDefaultCommand(turret.stop());

    // Copilot.x().whileTrue(turret.setAngle(Degrees.of(180)));
    // Copilot.y().whileTrue(turret.setAngle(Degrees.of(90)));
    // Copilot.b().whileTrue(turret.setAngle(Degrees.of(0)));
    // Copilot.rightBumper().whileTrue(turret.setDutyCycle(.25));
    // Copilot.leftBumper().whileTrue(turret.setDutyCycle(-.25));
    // Copilot.start().onTrue(turret.setAngle(() -> Degrees.of(220)));
    // Copilot.povUp().whileTrue(shooter.aimClockwise()).onFalse(shooter.aimStop());
    // Copilot.povDown().whileTrue(shooter.aimCounterClockwise()).onFalse(shooter.aimStop());

    // Copilot.povUp()
    //     .whileTrue(
    //         turret.setAngle(
    //             () ->
    //                 turret
    //                     .getAngle()
    //                     .plus(
    //                         Degrees.of(
    //                             10)))); // Continuously moves the turret up instead of just
    // moving
    // // 10 degrees from current position
    // // without the supplier it just sets it to 10 degrees instead of moving it up by 10 degrees
    // from
    // // current position?
    // Copilot.povDown().whileTrue(turret.setAngle(() -> turret.getAngle().minus(Degrees.of(10))));

    Copilot.povDown().whileTrue(climbElevator.setHeight(Inches.of(3)));
    Copilot.povUp().whileTrue(climbElevator.setHeight(Inches.of(5.5)));
    Copilot.povLeft().whileTrue(climbElevator.set(0.3));
    Copilot.povRight().whileTrue(climbElevator.set(-0.3));
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain
            .applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle));
  }

  private void RegisterNamedCommands() {}
}
