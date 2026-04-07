// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlanningConstants;
import frc.robot.commands.shootingCommands.feedWhenReady;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShotCalculator;
import frc.robot.subsystems.SpinnerIndex;
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
  public static final IntakePivot intakePivot = new IntakePivot();
  public static final Feeder feeder = new Feeder();
  public static final SpinnerIndex spindexer = new SpinnerIndex();
  public static final LEDS leds = new LEDS();
  public static final Turret turret = new Turret();
  public static final Shooter shooter = new Shooter();
  public static final Hood hood = new Hood();

  public static final ShotCalculator shotCalculator = new ShotCalculator(drivetrain);

  // Manual RPM setpoint for shooter tuning — D-pad up/down increments, Y runs it.
  public static double manualRPM = 4000.0;

  public Pose2d testPose = new Pose2d(2, 2, Rotation2d.fromDegrees(0));

  public RobotContainer() {
    // drivetrain.configNeutralMode(NeutralModeValue.Coast);

    DriverStation.silenceJoystickConnectionWarning(false); // TODO: Change to false for comps
    RegisterNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();

    SmartDashboard.putNumber("Shooter/Manual RPM", manualRPM);
    SmartDashboard.putNumber("Shooter/RPM Step", 250);
    SmartDashboard.putBoolean("Enable MegaTag2", false);
    SmartDashboard.putBoolean("Tuning/ShootOnTheMove", false);

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
                                    || Pilot.rightBumper().getAsBoolean()
                                ? (MaxSpeed * .2)
                                : MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -Pilot.getLeftX()
                            * (Pilot.leftBumper().getAsBoolean()
                                    || Pilot.rightBumper().getAsBoolean()
                                ? (MaxSpeed * .2)
                                : MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(
                        -Pilot.getRightX()
                            * (Pilot.leftBumper().getAsBoolean()
                                    || Pilot.rightBumper().getAsBoolean()
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

    Copilot.back().and(Copilot.rightStick()).onTrue(turret.zeroEncoder());

    // Copilot.start().whileTrue(shooter.sysId());

    // Copilot.a().whileTrue(turret.aimAtTarget().alongWith(shooter.set(.65)));
    Copilot.a()
        .whileTrue(
            turret
                .setAngle(shotCalculator::getIdealTurretAngle)
                .alongWith(shooter.setAngularVelocity(shotCalculator::getIdealShooterVelocity))
                .alongWith(hood.hoodUp().unless(this::isNearTrench))
                .alongWith(new feedWhenReady())); // aim + auto-feed when ready
    // Copilot.x()
    //     .whileTrue(
    //         feeder
    //             .runFeeder(() -> 0.5)
    //             .alongWith(hopperPush.runHopperPush(() -> -0.5))); // RUNS THROUGH ROBOT

    Copilot.y()
        .whileTrue(
            turret
                .setAngle(shotCalculator::getIdealTurretAngle)
                .alongWith(
                    shooter.setAngularVelocity(
                        () -> {
                          manualRPM = SmartDashboard.getNumber("Shooter/Manual RPM", manualRPM);
                          return RPM.of(manualRPM);
                        }))
                .alongWith(hood.hoodUp().unless(this::isNearTrench))
                .alongWith(new feedWhenReady()));

    // HOOD BUTTONS
    // Copilot.leftBumper().onTrue(new setHoodAngle(0));//Hood down
    // Copilot.leftBumper().onTrue(hood.setHoodPosition(0));
    // Copilot.rightBumper().onTrue(new setHoodAngle(1)); //Hood up
    // Copilot.rightBumper().onTrue(hood.setHoodPosition(1));
    Copilot.b().onTrue(hood.hoodUp());
    Copilot.x().onTrue(hood.hoodDown());

    //new Trigger(this::isNearTrench).onTrue(hood.hoodDown());
    // INTAKE, HOPPER, FEEDER

    intakePivot.setDefaultCommand(
        intakePivot.runDutyCycle(
            () ->
                -(Math.pow(Copilot.getLeftY(), 5)
                    + (0.25
                        * Copilot
                            .getLeftY())))); // x^5 control with linear control at smaller inputs.
    // Negative sign to correct for joystick direction.
    // intakePivot.setDefaultCommand(
    //     intakePivot.setAngle(() -> Degrees.of(90).times(Copilot.getLeftY())));
    Copilot.rightStick().toggleOnTrue((intakePivot.setAngle(Degrees.of(80))));
    Copilot.leftStick().toggleOnTrue(intakePivot.setAngle(Degrees.of(5)));
    // intakePivot.setDefaultCommand(intakePivot.stop());

    // Right bumper held = isolate spindexer only (mute intake roller)
    intakeRoller.setDefaultCommand(
        intakeRoller.runIntakeRollers(
            () ->
                Copilot.rightBumper().getAsBoolean()
                    ? 0.0
                    : .55
                        * (Copilot.getLeftTriggerAxis()
                            - Copilot.getRightTriggerAxis()))); // NEGATIVE RUNS THRU

    intakePivot.isInStoredPosition().whileTrue(intakeRoller.stopIntakeRollers());
    // Enable intake pivot zeroring on the fly
    Copilot.start()
        .and(Copilot.leftStick())
        .whileTrue(intakePivot.resetZeroToHardStop(Amps.of(40)));

    // Left bumper held = isolate intake only (mute spindexer)
    spindexer.setDefaultCommand(
        spindexer.runSpinnerIndex(
            () ->
                Copilot.leftBumper().getAsBoolean()
                    ? 0.0
                    : 0.4 * (Copilot.getRightTriggerAxis() - Copilot.getLeftTriggerAxis())));

    // feeder.setDefaultCommand(
    // feeder.runFeeder(() -> 0.5 * (Copilot.getRightTriggerAxis() -
    // Copilot.getLeftTriggerAxis()))); //Works
    feeder.setDefaultCommand(feeder.stopFeeder());

    // spindexer.setDefaultCommand(spindexer.stopSpinnerIndex());
    // POV left/right freed up — bumper isolation replaces independent spindexer control

    Copilot.povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double step = SmartDashboard.getNumber("Shooter/RPM Step", 250);
                  manualRPM += step;
                  SmartDashboard.putNumber("Shooter/Manual RPM", manualRPM);
                }));
    Copilot.povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double step = SmartDashboard.getNumber("Shooter/RPM Step", 250);
                  manualRPM -= step;
                  SmartDashboard.putNumber("Shooter/Manual RPM", manualRPM);
                }));

    Copilot.povLeft().onTrue(Commands.runOnce(() -> shotCalculator.logDataPoint(manualRPM)));
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
    NamedCommands.registerCommand(
        "Aim Turret", turret.setAngle(shotCalculator::getIdealTurretAngle));
    NamedCommands.registerCommand(
        "Spin Shooter To Target",
        shooter.setAngularVelocity(shotCalculator::getIdealShooterVelocity));
    NamedCommands.registerCommand("Stop Shooter", shooter.stop());
    NamedCommands.registerCommand("Stop Turret", turret.stop());
    // TODO: Rebuild the Spindexer commands and auto routines;
    NamedCommands.registerCommand("Stop Feeder Push", feeder.stopFeeder());
    NamedCommands.registerCommand("Stop Intake Pivot", intakePivot.stop());

    NamedCommands.registerCommand("Pivot Intake Out", intakePivot.runDutyCycle(.6));
    NamedCommands.registerCommand("Slow Pivot Intake Out", intakePivot.runDutyCycle(.3));
    NamedCommands.registerCommand("Pivot Intake In", intakePivot.runDutyCycle(-.6));
    NamedCommands.registerCommand("Intake", intakeRoller.runIntakeRollers(-.65));
    NamedCommands.registerCommand("Intake Stop", intakeRoller.runIntakeRollers(0));
    NamedCommands.registerCommand("Feeder Push", feeder.runFeeder(.5));
    NamedCommands.registerCommand("Hood Up", hood.hoodUp());
    NamedCommands.registerCommand("Hood Down", hood.hoodDown());
    NamedCommands.registerCommand("Spindexer In", spindexer.runSpinnerIndex(.4));
    NamedCommands.registerCommand("Spindexer Stop", spindexer.stopSpinnerIndex());
    // NamedCommands.registerCommand(null, getAutonomousCommand());
  }

  // Implements the following pseudocode:
  // Get current robot position
  // check if the robot position is within either a box or circle around any trench using
  // .contains()
  // Start with the blue alliance trench and then do use the fliputil like in shotcalculator to
  // get the red alliance trench locations
  // The trench locations can be found using the april tags locations
  //
  // ADD DIMENSIONS TO CONSTANTS
  // return true if the robot is within the defined area, false otherwise
  public boolean isNearTrench() {
    Pose2d robotPose = drivetrain.getState().Pose;

    Translation2d robotPositionBlue = robotPose.getTranslation();

    boolean near =
        FieldConstants.BLUEOUTPOST_ELLIPSE2D.contains(robotPositionBlue)
            || FieldConstants.BLUEHUMAN_ELLIPSE2D.contains(robotPositionBlue)
            || FieldConstants.REDOUTPOST_ELLIPSE2D.contains(robotPositionBlue)
            || FieldConstants.REDHUMAN_ELLIPSE2D.contains(robotPositionBlue);

    SmartDashboard.putBoolean("isNearTrench", near);
    return near;
  }
}
