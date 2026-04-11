package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.sensors.odometry.RobotOdometry;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotCommands {
  private final ShooterSubsystem shooterSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final IntakeRollerSubsystem intakeRollerSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final DriveSubsystem driveSubsystem;

  public RobotCommands(ShooterSubsystem shooterSubsystem, KickerSubsystem kickerSubsystem,
      SpindexerSubsystem spindexerSubsystem, IntakeSubsystem intakeSubsystem,
      IntakeRollerSubsystem intakeRollerSubsystem, HoodSubsystem hoodSubsystem, TurretSubsystem turretSubsystem,
      DriveSubsystem driveSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    this.hoodSubsystem = hoodSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  public void generateTriggers() {
    // new Trigger(() ->
    // shooterSubsystem.isJamDetected()).onTrue(unjamRoutineCommand());.
    new Trigger(() -> intakeRollerSubsystem.isJammed()).onTrue(intakeRollerUnjam());
  }

  private Command intakeRollerUnjam() {
    return intakeRollerSubsystem.runVoltageCommand(-5);
  }

  public Command unjamRoutineCommand() {
    // TODO: tune
    final double reverseVolts = 4.0;
    final double reverseTime = 0.25;

    return shooterSubsystem.stopCommand().alongWith(kickerSubsystem.stopCommand()).andThen(
        kickerSubsystem.runVoltageCommand(() -> -reverseVolts).withTimeout(reverseTime),
        kickerSubsystem.stopCommand());
  }

  public Command spindexerUnjamCommand() {
    return spindexerSubsystem.runVoltageCommand(() -> -6).until(() -> spindexerSubsystem.isJammed())
        .withTimeout(0.025);
  }

  public Command shootCommand() {
    ShotControl shotControl = ShotControl.getInstance();
    return shooterSubsystem.shootCommand()
        .alongWith(hoodSubsystem.runHoodToSetpointCommand(), kickerSubsystem.runCommand(),
            new InstantCommand(() -> shotControl.setShooting(true)),
            new WaitUntilCommand(() -> shooterSubsystem.isAtSetpoint() && hoodSubsystem.isAtSetpoint()
                && kickerSubsystem.isAtSetpoint()).andThen(spindexerSubsystem.runCommand())) // .alongWith(
        // new WaitCommand(2).andThen(intakeSubsystem.simpleOscillateIntakeCommand()))))
        .finallyDo(() -> {
          shotControl.setShooting(false);
          CommandScheduler.getInstance().schedule(hoodSubsystem.downCommand());
        });
  }

  public Command finishShootCommand() {
    return shooterSubsystem.shootCommand().alongWith(kickerSubsystem.runCommand()).withTimeout(0.5);
  }

  public Command bplShootCommand(double timeout) {
    ShotControl shotControl = ShotControl.getInstance();
    return shooterSubsystem.shootCommand().alongWith(hoodSubsystem.runHoodToSetpointCommand(),
        kickerSubsystem.runCommand(),
        new WaitUntilCommand(() -> shooterSubsystem.isAtSetpoint() && hoodSubsystem.isAtSetpoint()
            && kickerSubsystem.isAtSetpoint()).andThen(
                new InstantCommand(() -> shotControl.setShooting(true)),
                spindexerSubsystem.runCommand().alongWith(new WaitCommand(2)
                    .andThen(new InstantCommand(() -> CommandScheduler.getInstance()
                        .schedule(intakeRollerSubsystem.runVoltageCommand(-4)
                            .until(() -> !ShotControl.getInstance().isShooting())))))))
        .finallyDo(() -> shotControl.setShooting(false)).withTimeout(timeout);
  }

  public Command testShootCommand() {
    ShotControl shotControl = ShotControl.getInstance();
    return shooterSubsystem.runVelocityRPMCommand(() -> shooterSubsystem.getTestVelocity()).alongWith(
        kickerSubsystem.runCommand(),
        hoodSubsystem.setAngleDegCommand(() -> hoodSubsystem.getTestAngleDegrees()),
        new InstantCommand(() -> shotControl.setShooting(true)),
        new WaitUntilCommand(() -> shooterSubsystem.isAtTestSetpoint() && hoodSubsystem.isAtTestSetpoint()
            && kickerSubsystem.isAtSetpoint()).andThen(spindexerSubsystem.runCommand()))
        .finallyDo(() -> shotControl.setShooting(false));
  }

  public Command runIntakeCommand() {
    return intakeSubsystem.intakeDownCommand().alongWith(intakeRollerSubsystem.runCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command runReverseIntakeCommand() {
    return intakeSubsystem.intakeDownCommand().alongWith(intakeRollerSubsystem.runReverseCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command ferryCommand() {
    return runIntakeCommand().alongWith(shootCommand());
  }

  /*---------------
  | AUTO COMMANDS |
  ---------------*/

  public Command autoShootCommand() {
    ShotControl shotControl = ShotControl.getInstance();

    return shooterSubsystem.shootCommand()
        .alongWith(hoodSubsystem.runHoodToSetpointCommand(), kickerSubsystem.runCommand(),
            new InstantCommand(() -> shotControl.setShooting(true)),
            new WaitUntilCommand(() -> shooterSubsystem.isAtSetpoint() && hoodSubsystem.isAtSetpoint()
                && kickerSubsystem.isAtSetpoint())
                    .andThen(waitForShotCommand(),
                        spindexerSubsystem.runCommand()
                            .onlyWhile(() -> turretSubsystem.isAtSetpoint()).repeatedly()))
        .finallyDo(() -> {
          shotControl.setShooting(false);
          CommandScheduler.getInstance().schedule(hoodSubsystem.downCommand());
        });
  }

  public Command prepareShootCommand() {
    return shooterSubsystem.shootCommand().alongWith(kickerSubsystem.runCommand());
  }

  public Command autoIdleCommand() {
    return hoodSubsystem.downCommand().alongWith(shooterSubsystem.shootCommand());
  }

  public Command autoIntakeDownCommand() {
    return intakeSubsystem.runVoltageCommand(() -> -2).until(() -> intakeSubsystem.isDown())
        .andThen(intakeSubsystem.intakeHoldCommand());
  }

  public Command waitForTrustworthyPoseCommand() {
    return new WaitUntilCommand(() -> !RobotOdometry.instance.isDriveUntrustworthy("Main"));
  }

  public Command autoOscillateCommand(double waitTime) {
    return Commands.sequence(new WaitCommand(waitTime), intakeSubsystem.simpleOscillateIntakeCommand());
  }

  public Command autoOscillateCommand(double maxAngleDegrees, double timeout, double waitTime) {
    return Commands.sequence(new WaitCommand(waitTime),
        intakeSubsystem.simpleOscillateIntakeCommand(maxAngleDegrees, timeout));
  }

  public Command waitForShotCommand() {
    return new WaitUntilCommand(() -> turretSubsystem.isAtSetpoint());
  }

  public Command setSteerPositionCommand(Rotation2d rotation) {
    return driveSubsystem.setSteerPositionCommand(rotation);
  }
}
