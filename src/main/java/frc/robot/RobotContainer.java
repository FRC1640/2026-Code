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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
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
import frc.robot.subsystems.ShotControl.ShotType;
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

  // subsystems
  private DriveSubsystem driveSubsystem;
  private Gyro gyro;

  private TurretSubsystem turretSubsystem;

  private ShooterSubsystem shooterSubsystem;
  private HoodSubsystem hoodSubsystem;
  private KickerSubsystem kickerSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private SpindexerSubsystem spindexerSubsystem;
  private IntakeRollerSubsystem intakeRollerSubsystem;
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
    // custom formatting
    // create controllers
    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

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
    joystickDriveWeight = new JoystickDriveWeight(driveController::getLeftY, driveController::getLeftX,
        () -> -driveController.getRightX(), () -> driveController.rightBumper().getAsBoolean(),
        () -> driveController.leftBumper().getAsBoolean(), () -> true, gyro, () -> false);
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
    new ShotControl(() -> RobotOdometry.instance.getPose("Main"), () -> driveSubsystem.getChassisSpeeds(),
        ShotType.SCORING);
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
    driveController.start().onTrue(RobotOdometry.instance.resetGyroCommand(() -> new Rotation2d()));
    DriveWeightCommand.createWeightTrigger(driveToPointWeight, () -> driveController.a().getAsBoolean());
    DriveWeightCommand.createWeightTrigger(lockToPointWeight,
        () -> driveController.b().getAsBoolean()
            && (MathUtil.isNear(lockToPointWeight.getTargetPoint().getX(),
                lockToPointWeight.getRobotPose().getX(), LockToPointWeight.activeDistanceX))
            && (MathUtil.isNear(lockToPointWeight.getTargetPoint().getY(),
                lockToPointWeight.getRobotPose().getY(), LockToPointWeight.activeDistanceY)));
    operatorController.rightBumper().whileTrue(robotCommands.testShootCommand());
    operatorController.leftBumper().whileTrue(intakeRollerSubsystem.runVoltageCommand(-6));
    new Trigger(() -> Math.abs(operatorController.getLeftX()) > 0.1)
        .whileTrue(turretSubsystem.runVoltageCommand(() -> operatorController.getLeftX() * 2));
    driveController.pov(90).whileTrue(turretSubsystem.runVoltageCommand(() -> 2));
    driveController.pov(270).whileTrue(turretSubsystem.runVoltageCommand(() -> -2));
    // operatorController.pov(0).whileTrue(hoodSubsystem.setAngleRadCommand(() ->
    // HoodConstants.hoodAngle1Radians));
    operatorController.pov(0).whileTrue(hoodSubsystem.runVoltageCommand(() -> 1));
    operatorController.pov(180).whileTrue(hoodSubsystem.runVoltageCommand(() -> -1));
    operatorController.b().onTrue(intakeSubsystem.intakeDownCommand().until(() -> intakeSubsystem.isDown())
        .andThen(intakeSubsystem.intakeHoldCommand(IntakeConstants.activePositionRadians)));
    operatorController.a().whileTrue(intakeSubsystem.setPositionRadiansCommand(() -> Units.degreesToRadians(60)));
    new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.05)
        .whileTrue(intakeSubsystem.runVoltageCommand(() -> -operatorController.getLeftY() * 2));
    operatorController.y().whileTrue(intakeSubsystem.simpleOscillateIntakeCommand());
    operatorController.leftTrigger().whileTrue(new InstantCommand(() -> shooterSubsystem.incrementTestVelocity(-1))
        .andThen(new WaitCommand(0.02)).repeatedly());
    operatorController.rightTrigger().whileTrue(new InstantCommand(() -> shooterSubsystem.incrementTestVelocity(1))
        .andThen(new WaitCommand(0.02)).repeatedly());
    driveController.leftTrigger().toggleOnTrue(intakeRollerSubsystem.runCommand());
    driveController.rightTrigger()
        .whileTrue(new InstantCommand(() -> DriveWeightCommand.addWeight(shotCorrectionWeight))
            .andThen(new WaitUntilCommand(() -> shotCorrectionWeight.isDone()))
            .andThen(robotCommands.shootCommand()));
  }

  private void generateTriggers() {
    new Trigger(() -> bumpDetector.bumpDetected())
        .whileTrue(new RunCommand(() -> RobotOdometry.instance.distrustDrive("Main")));
    new Trigger(() -> AllianceManager
        .chooseFromAlliance(FieldConstants.blueAllianceZone, FieldConstants.redAllianceZone)
        .poseSatisfies(RobotOdometry.instance.getPose("Main")))
            .onTrue(new InstantCommand(() -> ShotControl.getInstance().setShotType(ShotType.SCORING)))
            .onFalse(new InstantCommand(() -> ShotControl.getInstance().setShotType(ShotType.FERRYING)));
    new Trigger(() -> DistanceManager.willPassPoint(
        DistanceManager.getNearestPosition(RobotOdometry.instance.getPose("Main"),
            AllianceManager.chooseFromAlliance(FieldConstants.blueTrenchCenters,
                FieldConstants.redTrenchCenters)),
        new Translation2d(1, 0), RobotOdometry.instance.getPose("Main"), driveSubsystem.getChassisSpeeds(), 1))
            .onTrue(new InstantCommand(() -> Logger.recordOutput("HoodAlmostSlammed", true))
                .andThen(hoodSubsystem.downCommand()));
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveSubsystem, () -> false));
    turretSubsystem.setDefaultCommand(turretSubsystem.trackCommand());
    hoodSubsystem.setDefaultCommand(hoodSubsystem.downCommand());
    // shooterSubsystem.setDefaultCommand(shooterSubsystem.runVelocityRPMCommand(()
    // -> 1500.0));
    // kickerSubsystem.setDefaultCommand(kickerSubsystem.stopCommand());
  }

  private void generateNamedCommands() {
    NamedCommands.registerCommand("DistrustOdometry", new InstantCommand(() -> {
      RobotOdometry.instance.distrustDrive("Main");
    }));
    NamedCommands.registerCommand("EnableAprilTags",
        new InstantCommand(() -> RobotOdometry.instance.setAutoApriltags(true)));
    NamedCommands.registerCommand("DisableAprilTags",
        new InstantCommand(() -> RobotOdometry.instance.setAutoApriltags(false)));
    NamedCommands.registerCommand("PrepareShoot", robotCommands.prepareAutoShootCommand());
    NamedCommands.registerCommand("Shoot", robotCommands.autoShootCommand());
    NamedCommands.registerCommand("ShooterIdle", robotCommands.autoIdleCommand());
    NamedCommands.registerCommand("WaitForTrustworthyPose",
        new WaitUntilCommand(() -> !RobotOdometry.instance.isDriveUntrustworthy("Main")));
    NamedCommands.registerCommand("IntakeDown",
        new InstantCommand(() -> CommandScheduler.getInstance().schedule(intakeSubsystem.intakeDownCommand())));
    NamedCommands.registerCommand("Intake", intakeRollerSubsystem.runCommand());
    NamedCommands.registerCommand("IntakeUP",
        new InstantCommand(() -> CommandScheduler.getInstance().schedule(intakeSubsystem.intakeUpCommand())));

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
