// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.WarningThresholdConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIO;
import frc.robot.sensors.apriltag.CameraConstant;
import frc.robot.sensors.gyro.BumpDetectorPeriodic;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.DriveToPoint;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.drive.weights.LockToPointWeight;
import frc.robot.subsystems.drive.weights.ShotCorrectionWeight;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.helpers.DistanceManager;
import frc.robot.util.logging.AlertsManager;
import frc.robot.util.logging.PeriodicLogging;
import frc.robot.util.motorDashboard.MotorDashboard;
import frc.robot.util.networktables.AutonChooser;
import frc.robot.util.periodic.PeriodicBase;
import frc.robot.util.periodic.PeriodicScheduler;
import frc.robot.util.projectileLogger.ProjectileLogger;
import frc.robot.util.sysid.SysIdChooser;

public class RobotContainer {

  // controllers
  private CommandXboxController driveController;
  private CommandXboxController operatorController;
  private CommandXboxController pitController;

  // subsystems
  private DriveSubsystem driveSubsystem;
  private Gyro gyro;

  private IntakeSubsystem intakeSubsystem;
  private IntakeRollerSubsystem intakeRollerSubsystem;

  private SpindexerSubsystem spindexerSubsystem;
  private KickerSubsystem kickerSubsystem;

  private ShooterSubsystem shooterSubsystem;
  private HoodSubsystem hoodSubsystem;
  private TurretSubsystem turretSubsystem;

  private ClimberSubsystem climberSubsystem;

  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();

  // drive weights
  private JoystickDriveWeight joystickDriveWeight;
  private DriveToPoint driveToPointWeight;
  private ShotCorrectionWeight shotCorrectionWeight;
  private LockToPointWeight lockToPointWeight;

  // dashboards
  private SysIdChooser sysIdChooser;
  private AutonChooser autonChooser;
  private ProjectileLogger projectileLogger;

  private PeriodicLogging periodicLogging;

  // other
  private RobotCommands robotCommands;
  private AlertsManager alertsManager;
  private BumpDetectorPeriodic bumpDetector;
  private MotorDashboard dashboard;

  public RobotContainer() {
    // create controllers
    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);
    pitController = new CommandXboxController(2);

    // create subsystems
    gyro = new Gyro(GyroIO.getIOByMode(() -> DriveConstants.kinematics
        .toChassisSpeeds(driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond));
    driveSubsystem = new DriveSubsystem(gyro);
    turretSubsystem = new TurretSubsystem(TurretSubsystem.getIOByMode());
    shooterSubsystem = new ShooterSubsystem(ShooterSubsystem.getIOByMode());
    hoodSubsystem = new HoodSubsystem(HoodSubsystem.getIOByMode());
    kickerSubsystem = new KickerSubsystem(KickerSubsystem.getIOByMode());
    spindexerSubsystem = new SpindexerSubsystem(SpindexerSubsystem.getIOByMode());
    intakeSubsystem = new IntakeSubsystem(IntakeSubsystem.getIOByMode());
    intakeRollerSubsystem = new IntakeRollerSubsystem(IntakeRollerSubsystem.getIOByMode());
    climberSubsystem = new ClimberSubsystem(ClimberSubsystem.getIOByMode());

    for (CameraConstant cameraConstant : RobotConstants.RobotInformation.robot.getCameras()) {
      aprilTagVisions.add(new AprilTagVision(AprilTagVisionIO.getIOByMode(cameraConstant,
          () -> new Pose3d(RobotOdometry.instance.getPose("Main"))), cameraConstant));
    }
    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);

    // create drive weights
    joystickDriveWeight = new JoystickDriveWeight(
        () -> -(!RobotState.isTest() ? driveController : pitController).getLeftY(),
        () -> -(!RobotState.isTest() ? driveController : pitController).getLeftX(),
        () -> -(!RobotState.isTest() ? driveController : pitController).getRightX(),
        () -> (!RobotState.isTest() ? driveController : pitController).leftBumper().getAsBoolean(),
        () -> (!RobotState.isTest() ? driveController : pitController).rightBumper().getAsBoolean(), () -> true,
        gyro, () -> false);
    DriveWeightCommand.addPersistentWeight(joystickDriveWeight);
    driveToPointWeight = new DriveToPoint(() -> RobotOdometry.instance.getPose("Main"), () -> new Pose2d(
        AllianceManager.chooseFromAlliance(FieldConstants.blueTowerBarCenter, FieldConstants.redTowerBarCenter),
        new Rotation2d()));
    lockToPointWeight = new LockToPointWeight(
        () -> RobotOdometry.instance.getPose("Main"), () -> DistanceManager
            .getNearestPosition(RobotOdometry.instance.getPose("Main"), FieldConstants.allTrenchCenters),
        LockToPointWeight.Y, Math.PI);

    // FieldConstants.blueTrenchCenters, FieldConstants.redTrenchCenters
    // general robot config
    bumpDetector = new BumpDetectorPeriodic(gyro, 3, Math.PI / 36);
    new RobotOdometry(driveSubsystem, gyro, visionArray).setBumpDetector(bumpDetector);
    robotCommands = new RobotCommands(shooterSubsystem, kickerSubsystem, spindexerSubsystem, intakeSubsystem,
        intakeRollerSubsystem, hoodSubsystem, turretSubsystem, driveSubsystem);
    alertsManager = new AlertsManager();
    AlertsManager.addAlert(() -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.", AlertType.kWarning);
    autonChooser = new AutonChooser();
    sysIdChooser = new SysIdChooser(driveSubsystem, shooterSubsystem, turretSubsystem, driveController);
    projectileLogger = new ProjectileLogger(robotCommands);

    periodicLogging = new PeriodicLogging();

    driveSubsystem.configurePathplanner();

    shotCorrectionWeight = new ShotCorrectionWeight(turretSubsystem);
    new ShotControl(() -> RobotOdometry.instance.getPose("Main"), () -> driveSubsystem.getChassisSpeeds());

    PeriodicScheduler.getInstance().addPeriodic(new PeriodicBase() {
      @Override
      public void periodic() {
        Logger.recordOutput("DistanceToHub",
            (FieldConstants.hubPositionRed
                .minus(RobotOdometry.instance.getPose("Main").plus(TurretConstants.turretTransform2d)))
                    .getTranslation().getNorm());
      }
    });

    configureBindings();
    generateTriggers();
    configureDefaultCommands();
    generateNamedCommands();
    loadResources();
  }

  private void configureBindings() {
    /*------------------
    | DRIVE CONTROLLER |
    ------------------*/

    driveController.start().onTrue(RobotOdometry.instance.resetGyroCommand(() -> new Rotation2d()));
    // DriveWeightCommand.createWeightTrigger(driveToPointWeight, () ->
    // driveController.a().getAsBoolean());
    DriveWeightCommand.createWeightTrigger(lockToPointWeight,
        () -> driveController.b().getAsBoolean()
            && (MathUtil.isNear(lockToPointWeight.getTargetPoint().getX(),
                lockToPointWeight.getRobotPose().getX(), LockToPointWeight.activeDistanceX))
            && (MathUtil.isNear(lockToPointWeight.getTargetPoint().getY(),
                lockToPointWeight.getRobotPose().getY(), LockToPointWeight.activeDistanceY)));

    driveController.leftTrigger()
        .toggleOnTrue(intakeRollerSubsystem.runCommand()
            .beforeStarting(() -> driveController.setRumble(RumbleType.kBothRumble, 0.5))
            .finallyDo(() -> driveController.setRumble(RumbleType.kBothRumble, 0.0)));
    driveController.rightTrigger().whileTrue(shootCommand());

    driveController.y()
        .toggleOnTrue(intakeSubsystem.intakeUpCommand()
            .until(() -> intakeSubsystem.isAtPosition(IntakeConstants.stowedPositionRadians))
            .andThen(intakeSubsystem.intakeHoldCommand(IntakeConstants.stowedPositionRadians)));

    driveController.pov(0).whileTrue(shootCommand().beforeStarting(() -> {
      ShotControl.getInstance().setManualSetpoint(ShotControl.towerManualSetpoint);
      ShotControl.getInstance().setManual(true);
    }).finallyDo(() -> ShotControl.getInstance().setManual(false)));
    driveController.pov(90).whileTrue(shootCommand().beforeStarting(() -> {
      ShotControl.getInstance().setManualSetpoint(ShotControl.leftTrenchManualSetpoint);
      ShotControl.getInstance().setManual(true);
    }).finallyDo(() -> ShotControl.getInstance().setManual(false)));
    driveController.pov(270).whileTrue(shootCommand().beforeStarting(() -> {
      ShotControl.getInstance().setManualSetpoint(ShotControl.rightTrenchManualSetpoint);
      ShotControl.getInstance().setManual(true);
    }).finallyDo(() -> ShotControl.getInstance().setManual(false)));

    /*---------------------
    | OPERATOR CONTROLLER |
    ---------------------*/

    // operatorController.pov(0).whileTrue(climberSubsystem.setHeightCommand(1));
    // operatorController.pov(180).whileTrue(climberSubsystem.runVoltageCommand(()
    // -> -6)); // TODO: IT IS IMPERATIVE THAT YOU TUNE THIS!!!!
    operatorController.rightBumper().whileTrue(robotCommands.unjamRoutineCommand());
    operatorController.leftBumper().whileTrue(robotCommands.runReverseIntakeCommand());

    operatorController.leftTrigger()
        .whileTrue(intakeSubsystem.runVoltageCommand(() -> -operatorController.getLeftY() * 2));

    operatorController.rightTrigger().whileTrue(intakeSubsystem.simpleOscillateIntakeCommand());

    /*----------------
    | PIT CONTROLLER |
    ----------------*/
    DriverStation.silenceJoystickConnectionWarning(true);
    pitController.pov(0).and(() -> RobotState.isTest()).whileTrue(hoodSubsystem.runVoltageCommand(() -> 2));
    pitController.pov(90).and(() -> RobotState.isTest()).whileTrue(turretSubsystem.runVoltageCommand(() -> 1.5));
    pitController.pov(180).and(() -> RobotState.isTest()).whileTrue(hoodSubsystem.runVoltageCommand(() -> -2));
    pitController.pov(270).and(() -> RobotState.isTest()).whileTrue(turretSubsystem.runVoltageCommand(() -> -1.5));

    pitController.a().and(() -> RobotState.isTest()).whileTrue(spindexerSubsystem.runCommand());
    pitController.rightTrigger().and(() -> RobotState.isTest())
        .whileTrue(intakeSubsystem.runVoltageCommand(() -> 2));
    pitController.leftTrigger().and(() -> RobotState.isTest())
        .whileTrue(intakeSubsystem.runVoltageCommand(() -> -2));
    pitController.leftBumper().and(() -> RobotState.isTest()).whileTrue(intakeRollerSubsystem.runCommand());
    pitController.rightBumper().and(() -> RobotState.isTest()).whileTrue(kickerSubsystem.runCommand());
    pitController.b().whileTrue(new InstantCommand(() -> shooterSubsystem.incrementTestVelocity(4))
        .andThen(new WaitCommand(0.02)).repeatedly());
    pitController.x().whileTrue(new InstantCommand(() -> shooterSubsystem.incrementTestVelocity(-4))
        .andThen(new WaitCommand(0.02)).repeatedly());
    pitController.y().whileTrue(robotCommands.testShootCommand());
  }

  private Command shootCommand() {
    return /*
         * new WaitUntilCommand(() ->
         * !RobotOdometry.instance.isDriveUntrustworthy("Main")) .deadlineFor(new
         * WaitCommand(0.3) .beforeStarting(() ->
         * driveController.setRumble(RumbleType.kBothRumble, 1.0)) .finallyDo(() ->
         * driveController.setRumble(RumbleType.kBothRumble, 0.0))) .andThen(
         */new InstantCommand(() -> DriveWeightCommand.addWeight(shotCorrectionWeight))
        .andThen(new WaitUntilCommand(() -> shotCorrectionWeight.isDone())
            .finallyDo(() -> DriveWeightCommand.removeWeight(shotCorrectionWeight)))
        .andThen(robotCommands.shootCommand());// );
  }

  private void generateTriggers() {
    new Trigger(() -> bumpDetector.bumpDetected())
        .whileTrue(new RunCommand(() -> RobotOdometry.instance.distrustDrive("Main")));
    new Trigger(() -> DistanceManager.willPassPoint(
        DistanceManager.getNearestPosition(RobotOdometry.instance.getPose("Main"),
            AllianceManager.chooseFromAlliance(FieldConstants.blueTrenchCenters,
                FieldConstants.redTrenchCenters)),
        new Translation2d(1, 0), RobotOdometry.instance.getPose("Main").plus(TurretConstants.turretTransform2d),
        driveSubsystem.getChassisSpeeds(), 1))
            .onTrue(new InstantCommand(() -> Logger.recordOutput("HoodAlmostSlammed", true))
                .andThen(hoodSubsystem.downCommand()));
    new Trigger(() -> FieldConstants.neutralZone.poseSatisfies(RobotOdometry.instance.getPose("Main")))
        .whileTrue(intakeRollerSubsystem.runCommand());
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveSubsystem, () -> false));
    turretSubsystem.setDefaultCommand(turretSubsystem.trackCommand());
    hoodSubsystem.setDefaultCommand(hoodSubsystem.downCommand());
    intakeSubsystem.setDefaultCommand(intakeSubsystem.intakeDownCommand().until(() -> intakeSubsystem.isDown())
        .andThen(intakeSubsystem.intakeHoldCommand(IntakeConstants.activePositionRadians)));
  }

  public void clearDefaultCommands(boolean clearDrive) {
    if (clearDrive) {
      CommandScheduler.getInstance().removeDefaultCommand(driveSubsystem);
    }
    CommandScheduler.getInstance().removeDefaultCommand(intakeSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(intakeRollerSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(spindexerSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(kickerSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(shooterSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(hoodSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(turretSubsystem);
    CommandScheduler.getInstance().removeDefaultCommand(climberSubsystem);
  }

  private void generateNamedCommands() {
    NamedCommands.registerCommand("DistrustOdometry", new InstantCommand(() -> {
      RobotOdometry.instance.distrustDrive("Main");
    }));
    NamedCommands.registerCommand("EnableAprilTags",
        new InstantCommand(() -> RobotOdometry.instance.setAutoApriltags(true)));
    NamedCommands.registerCommand("DisableAprilTags",
        new InstantCommand(() -> RobotOdometry.instance.setAutoApriltags(false)));
    NamedCommands.registerCommand("PrepareShoot", new InstantCommand());
    NamedCommands.registerCommand("Shoot", robotCommands.autoShootCommand());
    NamedCommands.registerCommand("ShooterIdle", robotCommands.autoIdleCommand());
    NamedCommands.registerCommand("WaitForTrustworthyPose",
        new WaitUntilCommand(() -> !RobotOdometry.instance.isDriveUntrustworthy("Main")));
    NamedCommands.registerCommand("IntakeDown", robotCommands.autoIntakeDownCommand());
    NamedCommands.registerCommand("Intake", intakeRollerSubsystem.runCommand());
    NamedCommands.registerCommand("IntakeUP",
        new InstantCommand(() -> CommandScheduler.getInstance().schedule(intakeSubsystem.intakeUpCommand())));
    NamedCommands.registerCommand("OscillateIntake", intakeSubsystem.simpleOscillateIntakeCommand(80));
  }

  public Command getAutonomousCommand() {
    return autonChooser.getAuto().finallyDo(() -> Logger.recordOutput("AutonDone", true));
  }

  public Command getBPLCommand() {
    return ProjectileLogger.bplCommand(robotCommands);
  }

  public void initializeDashboard() {
    dashboard = new MotorDashboard(kickerSubsystem, spindexerSubsystem, hoodSubsystem, shooterSubsystem,
        turretSubsystem, intakeSubsystem, intakeRollerSubsystem);
  }

  private void loadResources() {
    FieldConstants.getVisionSim();
    Logger.recordOutput("hide/turretLoad", new ShotControl.ShotSetpoint(0, 0, 0, 0));
  }
}
