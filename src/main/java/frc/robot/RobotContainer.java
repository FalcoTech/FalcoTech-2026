// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.shootingCommands.aimTurretAtTarget;
import frc.robot.commands.shootingCommands.alignAndShoot;
import frc.robot.commands.shootingCommands.feedWhenReady;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.HopperPush;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.IntakeSlide;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  // Drive speeds
  private static double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private static double MaxAngularRate =
      .4
          * RotationsPerSecond.of(0.75)
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
  public static final IntakeRoller intakeRoller = new IntakeRoller();
  public static final IntakeSlide intakeSlide = new IntakeSlide();
  public static final Feeder feeder = new Feeder();
  public static final HopperPush hopperPush = new HopperPush();
  public static final LEDS leds = new LEDS();
  public static final Turret turret = new Turret();
  public static final Shooter shooter = new Shooter();
  public static final ShotCalculator shotCalculator = new ShotCalculator(drivetrain);


  // Manual RPM setpoint for shooter tuning — D-pad up/down increments, Y runs it.
  public static double manualRPM = 4000.0;

  public Pose2d testPose = new Pose2d(2, 2, Rotation2d.fromDegrees(0));

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(false); // Changed to false for comps
    RegisterNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
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

    // DRIVETRAIN BUTTONS
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -Pilot.getLeftY()
                            * (Pilot.leftBumper().getAsBoolean()
                                ? (MaxSpeed * .2)
                                : MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -Pilot.getLeftX()
                            * (Pilot.leftBumper().getAsBoolean()
                                ? (MaxSpeed * .2)
                                : MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(
                        -Pilot.getRightX()
                            * (Pilot.leftBumper().getAsBoolean()
                                ? MaxAngularRate * .85
                                : MaxAngularRate)) // Drive counterclockwise with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    Pilot.a().whileTrue(drivetrain.applyRequest(() -> XForm));

    // Pilot.b().whileTrue(drivetrain.pathFindToPose(testPose));
    // Pilot.b().whileTrue(drivetrain.rotateThenPathfind(0, testPose));

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

    // TURRET AND SHOOTER BUTTONS
    turret.setDefaultCommand(turret.stop());
    shooter.setDefaultCommand(shooter.stop());

    // Copilot.start().whileTrue(shooter.sysId());

    // Copilot.a().whileTrue(turret.aimAtTarget().alongWith(shooter.set(.65)));
    Copilot.a()
        .whileTrue(
            new aimTurretAtTarget()
                .alongWith(
                    shooter.setAngularVelocity(() -> shotCalculator.getIdealShooterVelocity()))
                .alongWith(new feedWhenReady())); // aim + auto-feed when ready
    Copilot.x()
        .whileTrue(
            feeder
                .runFeeder(() -> 0.5)
                .alongWith(hopperPush.runHopperPush(() -> -0.5))); // RUNS THROUGH ROBOT
    Copilot.y()
        .whileTrue(
            new aimTurretAtTarget()
                .alongWith(shooter.setAngularVelocity(() -> RPM.of(manualRPM)))
                .alongWith(new feedWhenReady()));

    // INTAKE, HOPPER, FEEDER

    intakeSlide.setDefaultCommand(intakeSlide.runDutyCycle(() -> 0.6 * (Copilot.getLeftX())));

    intakeRoller.setDefaultCommand(
        intakeRoller.runIntakeRollers(
            () ->
                .65
                    * (Copilot.getLeftTriggerAxis()
                        - Copilot.getRightTriggerAxis()))); // NEGATIVE RUNS THRU

    // Copilot.rightBumper().whileTrue(intakeSlide.setHeight(Inches.of(10))); //Does not work
    // currently
    // Copilot.leftBumper().whileTrue(intakeSlide.setHeight(Inches.of(1))); //Does not work
    // currently

    // feeder.setDefaultCommand(
    // feeder.runFeeder(() -> 0.5 * (Copilot.getRightTriggerAxis() -
    // Copilot.getLeftTriggerAxis()))); //Works
    feeder.setDefaultCommand(feeder.stopFeeder());

    // hopperPush.setDefaultCommand(
    // hopperPush.runHopperPush(
    // () -> (Copilot.getLeftTriggerAxis() - Copilot.getRightTriggerAxis()) * .5)); //Works
    hopperPush.setDefaultCommand(hopperPush.stopHopperPush());

    // hopperPush.setDefaultCommand(hopperPush.runHopperPush(() -> Copilot.getLeftX()));

    Copilot.povUp().onTrue(Commands.runOnce(() -> manualRPM += 250));
    Copilot.povDown().onTrue(Commands.runOnce(() -> manualRPM -= 250));
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    // final var idle = new SwerveRequest.Idle();
    // return Commands.sequence(
    //     // Reset our field centric heading to match the robot
    //     // facing away from our alliance station wall (0 deg).
    //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
    //     // Then slowly drive forward (away from us) for 5 seconds.
    //     drivetrain
    //         .applyRequest(() -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
    //         .withTimeout(5.0),
    //     // Finally idle for the rest of auton
    //     drivetrain.applyRequest(() -> idle));

    return autoChooser.getSelected();

    // IF IT BREAKS TRY THIS:
    // try {
    //     return autoChooser.getSelected();
    // } catch (Exception e){
    //     return null;
    // }
  }

  private void RegisterNamedCommands() {
    NamedCommands.registerCommand("Aim Turret", new aimTurretAtTarget());
    NamedCommands.registerCommand(
        "Spin Shooter To Target",
        shooter.setAngularVelocity(() -> shotCalculator.getIdealShooterVelocity()));
    NamedCommands.registerCommand("Stop Shooter", shooter.stop());
    NamedCommands.registerCommand("Stop Turret", turret.stop());

    NamedCommands.registerCommand("Stop Hopper Push", hopperPush.stopHopperPush());
    NamedCommands.registerCommand("Stop Feeder Push", feeder.stopFeeder());
    NamedCommands.registerCommand("Stop Intake Slide", intakeSlide.stop());

    NamedCommands.registerCommand("Slide Intake Out", intakeSlide.runDutyCycle(.6));
    NamedCommands.registerCommand("Slow Slide Intake Out", intakeSlide.runDutyCycle(.3));
    NamedCommands.registerCommand("Slide Intake In", intakeSlide.runDutyCycle(-.6));
    NamedCommands.registerCommand("Intake", intakeRoller.runIntakeRollers(-.65));
    NamedCommands.registerCommand("Intake Stop", intakeRoller.runIntakeRollers(0));
    NamedCommands.registerCommand("Hopper Push", hopperPush.runHopperPush(-.5));
    NamedCommands.registerCommand("Feeder Push", feeder.runFeeder(.5));
    // NamedCommands.registerCommand(null, getAutonomousCommand());
  }
}
