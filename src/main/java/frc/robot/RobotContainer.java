// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CameraSettings;
import frc.robot.constants.RobotConstants.WarningThresholdConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.gyro.BumpDetectorPeriodic;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.DriveToPoint;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.deflector.DeflectorSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.util.helpers.AllianceManager;
import frc.robot.util.logging.AlertsManager;
import frc.robot.util.logging.PeriodicLogging;
import frc.robot.util.motorDashboard.Dashboard;
import frc.robot.util.networktables.AutonChooser;
import frc.robot.util.sysid.SysIdChooser;

public class RobotContainer {
  // controllers

  private CommandXboxController driveController;
  private CommandXboxController operatorController;

  // subsystems
  private DriveSubsystem driveSubsystem;
  private Gyro gyro;

  private TurretSubsystem turretSubsystem;

  private FlywheelSubsystem flywheelSubsystem;
  private DeflectorSubsystem deflectorSubsystem;
  private KickerSubsystem kickerSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private SpindexerSubsystem spindexerSubsystem;

  private BumpDetectorPeriodic bumpDetector;

  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();

  // drive weights
  private JoystickDriveWeight joystickDriveWeight;
  private DriveToPoint driveToPointWeight;
  // dashboards
  private SysIdChooser sysIdChooser;
  private AutonChooser autonChooser;

  private PeriodicLogging periodicLogging;

  // other
  private RobotCommands robotCommands;
  private AlertsManager alertsManager;

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
    flywheelSubsystem = new FlywheelSubsystem(FlywheelSubsystem.getIOByMode());
    deflectorSubsystem = new DeflectorSubsystem(DeflectorSubsystem.getIOByMode());
    kickerSubsystem = new KickerSubsystem(KickerSubsystem.getIOByMode());
    spindexerSubsystem = new SpindexerSubsystem(SpindexerSubsystem.getIOByMode());
    intakeSubsystem = new IntakeSubsystem(IntakeSubsystem.getIOByMode());

    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);

    // create drive weights
    joystickDriveWeight = new JoystickDriveWeight(driveController::getLeftY, driveController::getLeftX,
        () -> -driveController.getRightX(), () -> driveController.getRightTriggerAxis() > 0.1,
        () -> driveController.getLeftTriggerAxis() > 0.1, () -> true, gyro, () -> false);
    driveToPointWeight = new DriveToPoint(() -> RobotOdometry.instance.getPose("Main"), () -> new Pose2d(
        AllianceManager.chooseFromAlliance(FieldConstants.blueTowerBarCenter, FieldConstants.redTowerBarCenter),
        new Rotation2d()));
    DriveWeightCommand.addPersistentWeight(joystickDriveWeight);
    DriveWeightCommand.createWeightTrigger(driveToPointWeight, () -> driveController.a().getAsBoolean());

    // general robot config
    new RobotOdometry(driveSubsystem, gyro, visionArray);
    new ShooterControl(() -> RobotOdometry.instance.getPose("Main"), () -> driveSubsystem.getChassisSpeeds(),
        () -> ShooterControl.getNearestShootingPoint(RobotOdometry.instance.getPose("Main")),
        () -> gyro.getAngleRotation2d(), null);
    robotCommands = new RobotCommands(flywheelSubsystem, kickerSubsystem);
    alertsManager = new AlertsManager();
    AlertsManager.addAlert(() -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.", AlertType.kWarning);
    autonChooser = new AutonChooser();
    sysIdChooser = new SysIdChooser(driveSubsystem, flywheelSubsystem, turretSubsystem, driveController);
    bumpDetector = new BumpDetectorPeriodic(gyro, 3, Math.PI / 36);
    periodicLogging = new PeriodicLogging();

    // create drive weights
    joystickDriveWeight = new JoystickDriveWeight(driveController::getLeftX, () -> -driveController.getLeftY(),
        () -> -driveController.getRightX(), () -> driveController.getRightTriggerAxis() > 0.1,
        () -> driveController.getLeftTriggerAxis() > 0.1, () -> true, gyro, () -> false);
    DriveWeightCommand.addPersistentWeight(joystickDriveWeight);

    driveSubsystem.configurePathplanner();

    configureBindings();
    generateTriggers();
    configureDefaultCommands();
    generateNamedCommands();
    loadResources();
  }

  private void configureBindings() {
    driveController.start().onTrue(RobotOdometry.instance.resetGyroCommand(() -> new Rotation2d()));
  }

  private void generateTriggers() {
    new Trigger(() -> bumpDetector.bumpDetected()).onTrue(new InstantCommand(
        () -> RobotOdometry.instance.setVisionStdDevFactor("Main", CameraSettings.bumpVisionStdDevFactor)))
        .onFalse(new InstantCommand(() -> RobotOdometry.instance.resetVisionStdDevFactor("Main")));
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveSubsystem, () -> false));
    turretSubsystem.setDefaultCommand(turretSubsystem.trackCommand());
  }

  private void generateNamedCommands() {
  }

  public Command getAutonomousCommand() {
    return autonChooser.getAuto();
  }

  public void initializeDashboard() {
    new Dashboard(kickerSubsystem, spindexerSubsystem, deflectorSubsystem, flywheelSubsystem, turretSubsystem,
        intakeSubsystem);
  }

  private void loadResources() {
    FieldConstants.getVisionSim();
    Logger.recordOutput("hide/turretLoad", new ShooterControl.TurretSetpoint(0, 0, 0, 0));
  }
}
