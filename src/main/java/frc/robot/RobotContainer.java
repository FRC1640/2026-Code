// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants.CameraSettings;
import frc.robot.constants.RobotConstants.WarningThresholdConstants;
import frc.robot.sensors.apriltag.AprilTagVision;
import frc.robot.sensors.apriltag.AprilTagVisionIO;
import frc.robot.sensors.gyro.Gyro;
import frc.robot.sensors.gyro.GyroIO;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.subsystems.drive.weights.JoystickDriveWeight;
import frc.robot.subsystems.drive.weights.LocalTagAlignWeight;

import frc.robot.util.controller.PresetBoard;
import frc.robot.util.logging.alerts.AlertsManager;
import frc.robot.util.misc.AllianceManager;
import frc.robot.util.misc.DistanceManager;
import frc.robot.util.periodic.PeriodicBase;
import frc.robot.util.periodic.PeriodicScheduler;

public class RobotContainer {
  // controllers
  private CommandXboxController driveController;
  private CommandXboxController operatorController;
  private PresetBoard presetBoard;

  // subsystems

  private DriveSubsystem driveSubsystem;
  private Gyro gyro;

  private ArrayList<AprilTagVision> aprilTagVisions = new ArrayList<>();

  // drive weights
  private JoystickDriveWeight joystickDriveWeight;
  public LocalTagAlignWeight localAlignWeight;

  // autonomous
  private PathPlannerAuto autonomousCommand;

  // other
  RobotCommands robotCommands;
  AlertsManager alertsManager;

  public RobotContainer() {
    // create controllers
    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);
    presetBoard = new PresetBoard(2);

    // create subsystems
    gyro = new Gyro(GyroIO.getIOByMode(() -> DriveConstants.kinematics.toChassisSpeeds(
        driveSubsystem.getActualSwerveStates()).omegaRadiansPerSecond));
    driveSubsystem = new DriveSubsystem(gyro);

    /*
     * aprilTagVision = new AprilTagVision(
     * AprilTagVisionIO.getIOByMode(
     * CameraSettings.frontCameraLeft,
     * () -> new Pose3d(RobotOdometry.instance.getPose("Main"))),
     * CameraSettings.frontCameraLeft);
     */

    aprilTagVisions.add(new AprilTagVision(AprilTagVisionIO.getIOByMode(CameraSettings.reefCameraLeft,
        () -> new Pose3d(RobotOdometry.instance.getPose("Main"))), CameraSettings.reefCameraLeft));
    aprilTagVisions.add(new AprilTagVision(AprilTagVisionIO.getIOByMode(CameraSettings.reefCameraRight,
        () -> new Pose3d(RobotOdometry.instance.getPose("Main"))), CameraSettings.reefCameraRight));

    AprilTagVision[] visionArray = aprilTagVisions.toArray(AprilTagVision[]::new);

    // create drive weights
    joystickDriveWeight = new JoystickDriveWeight(
        driveController::getLeftX,
        () -> -driveController.getLeftY(),
        () -> -driveController.getRightX(),
        () -> driveController.getRightTriggerAxis() > 0.1,
        () -> driveController.getLeftTriggerAxis() > 0.1,
        () -> true,
        gyro,
        () -> false);
    DriveWeightCommand.addPersistentWeight(joystickDriveWeight);

    localAlignWeight = new LocalTagAlignWeight(
        () -> DistanceManager.getNearestPosition(
            RobotOdometry.instance.getPose("Main"),
            AllianceManager.chooseFromAlliance(
                FieldConstants.reefPositionsBlue, FieldConstants.reefPositionsRed)),
        () -> RobotOdometry.instance.getPose("Main").getRotation(),
        driveSubsystem,
        gyro,
        visionArray);

    // general robot config
    new RobotOdometry(driveSubsystem, gyro, visionArray);

    alertsManager = new AlertsManager();
    AlertsManager.addAlert(
        () -> RobotController.getBatteryVoltage() < WarningThresholdConstants.minBatteryVoltage,
        "Low battery voltage.",
        AlertType.kWarning);

    driveSubsystem.configurePathplanner();
    robotCommands.generateTriggers();

    configureBindings();
    configureDefaultCommands();
    generateNamedCommands();
    loadResources();
    // create autonomous command
    autonomousCommand = new PathPlannerAuto("Center Barge Auton ");

    PeriodicScheduler.getInstance().addPeriodic(new PeriodicBase() {
      @Override
      public void periodic() {

      }
    });
  }

  private void configureBindings() {
    /*------------------
    | DRIVE CONTROLLER |
    ------------------*/

    /*---------------------
    | OPERATOR CONTROLLER |
    ---------------------*/

    /*--------------
    | PRESET BOARD |
    --------------*/
    // Example:
    // new Trigger(() -> presetBoard.getLl2())
    // .onTrue(new InstantCommand(() ->
    // robotCommands.setPreset(CoralPreset.L2_LEFT)));

    // non-controller triggers

    // reset gyro
    new Trigger(() -> Robot.getState() == RobotState.TELEOP).onTrue(new InstantCommand(
        () -> {
          /*
           * Rotation2d gyroResetRotLogged = autonomousCommand.getStartingPose()
           * .getRotation().minus(RobotOdometry.instance.getPose("Main")
           * .getRotation());
           * gyro.setOffset(gyroResetRotLogged.getRadians());
           */
          gyro.addOffset(
              autonomousCommand.getStartingPose().getRotation() // previous reference angle
                  .minus(Rotation2d.kCCW_Pi_2) // convert to climber side
                  .minus(Rotation2d.kPi) // reset direction (climber side)
                  .getRadians());
        }));

  }

  private void configureDefaultCommands() {
    driveSubsystem.setDefaultCommand(
        DriveWeightCommand.create(driveSubsystem, () -> false));
  }

  private void generateNamedCommands() {
    // Example: NamedCommands.registerCommand("SetupL4",
    // robotCommands.setupPoleCommand(CoralPreset.L4_LEFT));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Jaketron")
        .finallyDo(() -> Logger.recordOutput("AutoDone", false))
        .handleInterrupt(() -> Logger.recordOutput("AutoDone", true));
  }

  private void loadResources() {
    FieldConstants.getVisionSim();
  }
}
