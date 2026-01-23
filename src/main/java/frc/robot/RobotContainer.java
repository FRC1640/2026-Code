// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.WarningThresholdConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.deflector.DeflectorIO;
import frc.robot.subsystems.shooter.deflector.DeflectorSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.util.logging.AlertsManager;

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

  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();

  // drive weights
  private JoystickDriveWeight joystickDriveWeight;

  // other
  RobotCommands robotCommands;
  AlertsManager alertsManager;

  public RobotContainer() {
    // create controllers
    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    // create subsystems
    gyro = new Gyro(GyroIO.getIOByMode(() -> DriveConstants.kinematics
        .toChassisSpeeds(driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond));
    driveSubsystem = new DriveSubsystem(gyro);
    turretSubsystem = new TurretSubsystem(TurretIO.getIOByMode());
    flywheelSubsystem = new FlywheelSubsystem(FlywheelIO.getIOByMode());
    deflectorSubsystem = new DeflectorSubsystem(DeflectorIO.getIOByMode());

    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);

    // create drive weights
    joystickDriveWeight = new JoystickDriveWeight(driveController::getLeftX, () -> -driveController.getLeftY(),
        () -> -driveController.getRightX(), () -> driveController.getRightTriggerAxis() > 0.1,
        () -> driveController.getLeftTriggerAxis() > 0.1, () -> true, gyro, () -> false);
    DriveWeightCommand.addPersistentWeight(joystickDriveWeight);

    // general robot config
    new RobotOdometry(driveSubsystem, gyro, visionArray);
    new ShooterControl(
      () -> RobotOdometry.instance.getPose("Main"),
      () -> driveSubsystem.getChassisSpeeds(),
      () -> new Pose2d()); // TODO target pose (add to field constants)
    robotCommands = new RobotCommands();
    alertsManager = new AlertsManager();
    AlertsManager.addAlert(() -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.", AlertType.kWarning);

    driveSubsystem.configurePathplanner();
    robotCommands.generateTriggers();

    configureBindings();
    configureDefaultCommands();
    generateNamedCommands();
    loadResources();
  }

  private void configureBindings() {
  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(DriveWeightCommand.create(driveSubsystem, () -> false));
    turretSubsystem.setDefaultCommand(turretSubsystem.trackCommand());
  }

  private void generateNamedCommands() {
  }

  public Command getAutonomousCommand() {
    return new PrintCommand("No autonomous command configured");
  }

  private void loadResources() {
    FieldConstants.getVisionSim();
    Logger.recordOutput("hide/turretLoad", new ShooterControl.TurretSetpoint(0, 0, 0, 0));
  }
}
