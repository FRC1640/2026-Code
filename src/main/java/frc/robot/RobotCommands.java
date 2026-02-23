package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.ShotControl;
import frc.robot.subsystems.ShotControl.ShotType;
import frc.robot.subsystems.ShotControl.TurretSetpoint;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

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
    new Trigger(() -> shooterSubsystem.isJamDetected()).onTrue(unjamRoutineCommand());
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
    return shooterSubsystem.shootCommand()
        .alongWith(hoodSubsystem.runHoodToSetpointCommand(),
            new InstantCommand(() -> shotControl.setShooting(true)),
            new WaitUntilCommand(() -> shooterSubsystem.isAtSetpoint() && hoodSubsystem.isAtSetpoint())
                .andThen(kickerSubsystem.runCommand()
                    .alongWith(new WaitUntilCommand(() -> kickerSubsystem.isAtSetpoint())
                        .andThen(spindexerSubsystem.runCommand()))))
        .finallyDo(() -> shotControl.setShooting(false));
  }

  public Command runIntakeCommand() {
    return intakeSubsystem.intakeDownCommand().alongWith(intakeRollerSubsystem.runCommand());
  }

  public Command ferryCommand() {
    return runIntakeCommand().alongWith(shootCommand());
  }

  // BALL PROJECTILE LOGGER COMMAND
  public Command bplCommand(double shooterVelocityRPM0, double shooterVelocityRPMf, double RPMStep,double hoodAngleDeg0, double hoodAngleDegf, double DegStep) {
    int shooterSteps = (int)Math.ceil(shooterVelocityRPMf - shooterVelocityRPM0 / RPMStep);
    int hoodSteps = (int)Math.ceil(hoodAngleDeg0 - hoodAngleDegf / DegStep);
    
    ArrayList<Command> commands = new ArrayList<Command>(shooterSteps * hoodSteps);
    for (double shooterVelocityRPM = shooterVelocityRPM0; shooterVelocityRPM <= shooterVelocityRPMf; shooterVelocityRPM++) {
      ShotControl.getInstance().setShotType(ShotType.MANUAL);
      final double localshooterVelocityRPM = shooterVelocityRPM;
      for (double hoodAngleDeg = hoodAngleDeg0; hoodAngleDeg <= hoodAngleDegf; hoodAngleDeg++) {
        final double localHoodAngleDeg = hoodAngleDeg;  // mutable variables cannot be used in lambdas.
        commands.add(new InstantCommand(() -> {ShotControl.getInstance().setSetpoint(new TurretSetpoint(0, 0, localHoodAngleDeg, localshooterVelocityRPM));}).andThen(shootCommand(), new WaitCommand(1)));
      }
    }

    return new SequentialCommandGroup((Command[])commands.toArray());
  }
}
