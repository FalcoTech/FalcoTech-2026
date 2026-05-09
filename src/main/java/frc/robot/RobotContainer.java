// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
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
import edu.wpi.first.math.MathUtil;
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
import frc.robot.Constants.HoodConstants;
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
import frc.robot.util.HubShiftUtil;
import java.util.Set;

public class RobotContainer {
  // Drive speeds
  private static double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private static double MaxAngularRate =
      .7
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
  private final CommandXboxController TestController = new CommandXboxController(5);

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
  private static double testHoodSetpoint = HoodConstants.HOOD_UP;
  private boolean testSlowMode = false;

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
    SmartDashboard.putNumber("Testing/Hood Step", 0.1);
    SmartDashboard.putNumber("Testing/RPM Step", 250.0);
    SmartDashboard.putNumber("Testing/Hood Setpoint", testHoodSetpoint);
    SmartDashboard.putBoolean("Testing/Slow Mode", testSlowMode);
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

    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));

    Pilot.a().whileTrue(drivetrain.applyRequest(() -> XForm));

    Pilot.b().onTrue(Commands.runOnce(() -> drivetrain.setDriveBoost(true)));
    Pilot.b().onFalse(Commands.runOnce(() -> drivetrain.setDriveBoost(false)));

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
                .alongWith(hood.positionCommand(shotCalculator::getIdealHoodPosition))
                .alongWith(new feedWhenReady())
                .withName("Auto Aim & Fire")); // aim + auto-feed when ready
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
                .alongWith(hood.positionCommand(shotCalculator::getIdealHoodPosition))
                .alongWith(new feedWhenReady())
                .withName("Manual RPM & Fire"));

    // HOOD BUTTONS
    Copilot.b().onTrue(hood.hoodUp());
    Copilot.x().onTrue(hood.hoodDown());
    Copilot.x().and(Copilot.back()).onTrue(hood.positionCommand(0.25));
    // Copilot.b().onTrue(new setHoodAngle(0.5));
    // Copilot.x().onTrue(new setHoodAngle(0));
    new Trigger(this::isNearTrench)
        .and(RobotModeTriggers.teleop())
        .whileTrue(hood.hoodDown().repeatedly());
    // INTAKE, HOPPER, FEEDER

    // RobotModeTriggers.teleop()
    //     .whileTrue(
    //         intakePivot.runDutyCycle(
    //             () ->
    //                 -0.75*(Math.pow(Copilot.getLeftY(), 5)
    //                     + (0.25
    //                         * Copilot
    //                             .getLeftY())))); // x^5 control with linear control at smaller
    // inputs.
    // Negative sign to correct for joystick direction.

    intakePivot.setDefaultCommand(
        Commands.either(
            intakePivot.runDutyCycle(
                () -> -0.75 * (Math.pow(Copilot.getLeftY(), 5) + (0.25 * Copilot.getLeftY()))),
            Commands.idle(intakePivot),
            DriverStation::isTeleop));
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
                    : 1
                        * (Copilot.getLeftTriggerAxis()
                            - Copilot.getRightTriggerAxis()))); // NEGATIVE RUNS THRU

    intakePivot
        .isInStoredPosition()
        .and(RobotModeTriggers.teleop())
        .whileTrue(intakeRoller.stopIntakeRollers());
    // Enable intake pivot zeroring on the fly
    Copilot.start().and(Copilot.leftStick()).whileTrue(intakePivot.resetEncoderToLimit());
    // .whileTrue(intakePivot.resetZeroToHardStop(Amps.of(40)));

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

    Copilot.povLeft()
        .onTrue(Commands.runOnce(() -> shotCalculator.logDataPoint(manualRPM, testHoodSetpoint)));

    configureTestBindings();
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
        "Aim Turret",
        Commands.defer(() -> turret.setAngle(shotCalculator::getIdealTurretAngle), Set.of(turret)));
    NamedCommands.registerCommand(
        "Spin Shooter To Target",
        Commands.defer(
            () -> shooter.setAngularVelocity(shotCalculator::getIdealShooterVelocity),
            Set.of(shooter)));
    NamedCommands.registerCommand(
        "Stop Shooter", Commands.defer(() -> shooter.stop(), Set.of(shooter)));
    NamedCommands.registerCommand(
        "Stop Turret", Commands.defer(() -> turret.stop(), Set.of(turret)));
    NamedCommands.registerCommand(
        "Stop Feeder Push", Commands.defer(() -> feeder.stopFeeder(), Set.of(feeder)));
    NamedCommands.registerCommand(
        "Stop Intake Pivot", Commands.defer(() -> intakePivot.stop(), Set.of(intakePivot)));
    NamedCommands.registerCommand(
        "Pivot Intake Out",
        Commands.defer(
            () -> intakePivot.setAngleAndStop(Degree.of(0), Degrees.of(5)), Set.of(intakePivot)));
    NamedCommands.registerCommand(
        "Slow Pivot Intake Out",
        Commands.defer(() -> intakePivot.runDutyCycle(.3), Set.of(intakePivot)));
    NamedCommands.registerCommand(
        "Pivot Intake In",
        Commands.defer(
            () -> intakePivot.setAngleAndStop(Degree.of(100), Degrees.of(5)), Set.of(intakePivot)));
    NamedCommands.registerCommand(
        "Intake", Commands.defer(() -> intakeRoller.runIntakeRollers(-.65), Set.of(intakeRoller)));
    NamedCommands.registerCommand(
        "Intake Stop",
        Commands.defer(() -> intakeRoller.runIntakeRollers(0), Set.of(intakeRoller)));
    NamedCommands.registerCommand(
        "Feeder Push", Commands.defer(() -> new feedWhenReady(), Set.of(feeder)));
    NamedCommands.registerCommand("Hood Up", Commands.defer(() -> hood.hoodUp(), Set.of(hood)));
    NamedCommands.registerCommand("Hood Down", Commands.defer(() -> hood.hoodDown(), Set.of(hood)));
    NamedCommands.registerCommand(
        "Spindexer In", Commands.defer(() -> spindexer.runSpinnerIndex(.4), Set.of(spindexer)));
    NamedCommands.registerCommand(
        "Spindexer Stop", Commands.defer(() -> spindexer.stopSpinnerIndex(), Set.of(spindexer)));
    NamedCommands.registerCommand("Dumb Feeder", feeder.runFeeder(.8));
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

  private boolean isGatingValid() {
    boolean turretReady =
        turret
            .getAngleSetpoint()
            .map(sp -> turret.isNearAngle(sp, Degrees.of(3)).getAsBoolean())
            .orElse(false);
    boolean shooterReady =
        shooter
            .getAngularVelocitySetpoint()
            .map(sp -> shooter.isNearVelocity(sp, RPM.of(150)).getAsBoolean())
            .orElse(false);
    return turretReady && shooterReady;
  }

  private void configureTestBindings() {
    // DRIVETRAIN — medium speed (0.6), slow toggle (0.2)
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -TestController.getLeftY()
                            * (testSlowMode ? MaxSpeed * 0.2 : MaxSpeed * 0.6))
                    .withVelocityY(
                        -TestController.getLeftX()
                            * (testSlowMode ? MaxSpeed * 0.2 : MaxSpeed * 0.6))
                    .withRotationalRate(
                        -TestController.getRightX()
                            * (testSlowMode ? MaxAngularRate * 0.4 : MaxAngularRate * 0.85))));

    // HOOD — always tracks testHoodSetpoint
    hood.setDefaultCommand(hood.run(() -> hood.setPosition(testHoodSetpoint)));

    // INTAKE PIVOT — right stick Y with x^5 curve (same formula as copilot)
    intakePivot.setDefaultCommand(
        Commands.either(
            intakePivot.runDutyCycle(
                () ->
                    -0.75
                        * (Math.pow(TestController.getRightY(), 5)
                            + (0.25 * TestController.getRightY()))),
            Commands.idle(intakePivot),
            DriverStation::isTeleop));

    // INTAKE ROLLER — left trigger in, right trigger reverse (no bumper isolation)
    intakeRoller.setDefaultCommand(
        intakeRoller.runIntakeRollers(
            () -> TestController.getLeftTriggerAxis() - TestController.getRightTriggerAxis()));

    // SPINDEXER — right trigger in, left trigger reverse (same formula as copilot)
    spindexer.setDefaultCommand(
        spindexer.runSpinnerIndex(
            () ->
                0.4
                    * (TestController.getRightTriggerAxis()
                        - TestController.getLeftTriggerAxis())));

    // A — manual RPM + turret auto-aim + spindexer creep when gating valid
    TestController.a()
        .whileTrue(
            turret
                .setAngle(shotCalculator::getIdealTurretAngle)
                .alongWith(shooter.setAngularVelocity(() -> RPM.of(manualRPM)))
                .alongWith(spindexer.runSpinnerIndex(() -> isGatingValid() ? 0.2 : 0.0))
                .withName("Test Manual RPM & Fire"));

    // B (alone) — snap manualRPM to shot calculator table value for current distance
    TestController.b()
        .and(TestController.back().negate())
        .onTrue(
            Commands.runOnce(
                () -> {
                  manualRPM = shotCalculator.getIdealShooterVelocity().in(RPM);
                  SmartDashboard.putNumber("Shooter/Manual RPM", manualRPM);
                }));

    // X (alone) — reset testHoodSetpoint to HOOD_UP baseline
    TestController.x()
        .and(TestController.back().negate())
        .onTrue(
            Commands.runOnce(
                () -> {
                  testHoodSetpoint = HoodConstants.HOOD_UP;
                  SmartDashboard.putNumber("Testing/Hood Setpoint", testHoodSetpoint);
                }));

    // Y — full auto aim using shot calculator (mirrors copilot A)
    TestController.y()
        .whileTrue(
            turret
                .setAngle(shotCalculator::getIdealTurretAngle)
                .alongWith(shooter.setAngularVelocity(shotCalculator::getIdealShooterVelocity))
                .alongWith(hood.positionCommand(shotCalculator::getIdealHoodPosition))
                .alongWith(new feedWhenReady())
                .withName("Test Auto Aim & Fire"));

    // NOTE: Back must be pressed before B/X for the negate() guard to prevent the solo
    // B/X actions from also firing. For test use, this is acceptable — both actions are safe.
    // Back+B — turret encoder zero
    TestController.back().and(TestController.b()).onTrue(turret.zeroEncoder());

    // Back+X — intakePivot encoder reset to limit switch
    TestController.back().and(TestController.x()).onTrue(intakePivot.resetEncoderToLimit());

    // D-pad up — hood setpoint increase
    TestController.povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double step = SmartDashboard.getNumber("Testing/Hood Step", 0.1);
                  testHoodSetpoint = MathUtil.clamp(testHoodSetpoint + step, 0.0, 1.0);
                  SmartDashboard.putNumber("Testing/Hood Setpoint", testHoodSetpoint);
                }));

    // D-pad down — hood setpoint decrease
    TestController.povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double step = SmartDashboard.getNumber("Testing/Hood Step", 0.1);
                  testHoodSetpoint = MathUtil.clamp(testHoodSetpoint - step, 0.0, 1.0);
                  SmartDashboard.putNumber("Testing/Hood Setpoint", testHoodSetpoint);
                }));

    // D-pad right — manual RPM increase
    TestController.povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double step = SmartDashboard.getNumber("Testing/RPM Step", 250.0);
                  manualRPM += step;
                  SmartDashboard.putNumber("Shooter/Manual RPM", manualRPM);
                }));

    // D-pad left — manual RPM decrease
    TestController.povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  double step = SmartDashboard.getNumber("Testing/RPM Step", 250.0);
                  manualRPM -= step;
                  SmartDashboard.putNumber("Shooter/Manual RPM", manualRPM);
                }));

    // Right stick click — log shot data point to console and SmartDashboard
    TestController.rightStick()
        .onTrue(Commands.runOnce(() -> shotCalculator.logDataPoint(manualRPM, testHoodSetpoint)));

    // Left stick click — toggle slow drive mode
    TestController.leftStick()
        .onTrue(
            Commands.runOnce(
                () -> {
                  testSlowMode = !testSlowMode;
                  SmartDashboard.putBoolean("Testing/Slow Mode", testSlowMode);
                }));

    // Right bumper — manual feeder push while held
    TestController.rightBumper().whileTrue(feeder.runFeeder(0.8));

    // Start — seed field-centric heading
    TestController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
  }
}
