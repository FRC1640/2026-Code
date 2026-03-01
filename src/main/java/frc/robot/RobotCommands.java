package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
    // shooterSubsystem.isJamDetected()).onTrue(unjamRoutineCommand());
  }

  private Command unjamRoutineCommand() {
    // TODO: tune
    final double reverseVolts = 4.0;
    final double reverseTime = 0.25;

    return shooterSubsystem.stopCommand().alongWith(kickerSubsystem.stopCommand()).andThen(
        kickerSubsystem.runVoltageCommand(() -> -reverseVolts).withTimeout(reverseTime),
        kickerSubsystem.stopCommand());
  }

  public Command shootCommand() {
    ShotControl shotControl = ShotControl.getInstance();
    return shooterSubsystem.shootCommand().alongWith(hoodSubsystem.runHoodToSetpointCommand(),
        kickerSubsystem.runCommand(), new InstantCommand(() -> shotControl.setShooting(true)),
        new WaitUntilCommand(() -> shooterSubsystem.isAtSetpoint() && hoodSubsystem.isAtSetpoint()
            && kickerSubsystem.isAtSetpoint())
                .andThen(spindexerSubsystem.runCommand()
                    /*.alongWith(new WaitCommand(2).andThen(new InstantCommand(() -> CommandScheduler
                        .getInstance()
                        .schedule(intakeSubsystem
                            .oscillateIntakeCommand(Units.degreesToRadians(35),
                                Units.degreesToRadians(20), 2)
                            .alongWith(intakeRollerSubsystem.runVoltageCommand(-4))
                            .until(() -> !ShotControl.getInstance().isShooting())))))*/))
        .finallyDo(() -> shotControl.setShooting(false));
  }

  public Command bplShootCommand(double timeout) {
    ShotControl shotControl = ShotControl.getInstance();
    return shooterSubsystem.shootCommand().alongWith(hoodSubsystem.runHoodToSetpointCommand(),
        kickerSubsystem.runCommand(), new InstantCommand(() -> shotControl.setShooting(true)),
        new WaitUntilCommand(() -> shooterSubsystem.isAtSetpoint() && hoodSubsystem.isAtSetpoint()
            && kickerSubsystem.isAtSetpoint()).andThen(spindexerSubsystem.runCommand().alongWith(
                new WaitCommand(2).andThen(new InstantCommand(() -> CommandScheduler.getInstance()
                    .schedule(intakeRollerSubsystem.runVoltageCommand(-4)
                        .until(() -> !ShotControl.getInstance().isShooting())))))))
        .finallyDo(() -> shotControl.setShooting(false)).withTimeout(timeout);
  }

  public Command testShootCommand() {
    ShotControl shotControl = ShotControl.getInstance();
    return shooterSubsystem.runVelocityRPMCommand(() -> shooterSubsystem.getTestVelocity()).alongWith(
        kickerSubsystem.runCommand(), new InstantCommand(() -> shotControl.setShooting(true)),
        new WaitUntilCommand(() -> shooterSubsystem.isAtTestSetpoint() // && hoodSubsystem.isAtSetpoint()
            && kickerSubsystem.isAtSetpoint())
                .andThen(spindexerSubsystem.runCommand().alongWith(new WaitCommand(2)
                    .andThen(new InstantCommand(() -> CommandScheduler.getInstance().schedule(
                        /*
                         * intakeSubsystem .oscillateIntakeCommand(Units.degreesToRadians(35),
                         * Units.degreesToRadians(20), 2)
                         */Commands.none()
                            .alongWith(intakeRollerSubsystem.runVoltageCommand(-4))
                            .until(() -> !ShotControl.getInstance().isShooting())))))))
        .finallyDo(() -> shotControl.setShooting(false));
  }

  public Command runIntakeCommand() {
    return intakeSubsystem.intakeDownCommand().alongWith(intakeRollerSubsystem.runCommand())
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command ferryCommand() {
    return runIntakeCommand().alongWith(shootCommand());
  }

  /*---------------
  | AUTO COMMANDS |
  ---------------*/

  public Command prepareAutoShootCommand() {
    return new InstantCommand(() -> CommandScheduler.getInstance()
        .schedule(hoodSubsystem.runHoodToSetpointCommand().alongWith(shooterSubsystem.shootCommand())));
  }

  public Command autoShootCommand() {
    return kickerSubsystem.runCommand()
        .alongWith(new WaitUntilCommand(() -> kickerSubsystem.isAtSetpoint())
            .andThen(spindexerSubsystem.runCommand())
            .alongWith(new InstantCommand(() -> CommandScheduler.getInstance().schedule(intakeSubsystem
                .oscillateIntakeCommand(Units.degreesToRadians(25), Units.degreesToRadians(10), 2)))));
  }

  public Command autoIdleCommand() {
    return new InstantCommand(() -> CommandScheduler.getInstance()
        .schedule(hoodSubsystem.downCommand().alongWith(shooterSubsystem.runVelocityRPMCommand(() -> 1500))));
  }
}
