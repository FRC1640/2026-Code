package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakeRollers.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.deflector.DeflectorSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class RobotCommands {
  private final FlywheelSubsystem flywheelSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final IntakeRollerSubsystem intakeRollerSubsystem;
  private final DeflectorSubsystem deflectorSubsystem;
  private final TurretSubsystem turretSubsystem;
  private final DriveSubsystem driveSubsystem;

  public RobotCommands(FlywheelSubsystem flywheelSubsystem, KickerSubsystem kickerSubsystem,
      SpindexerSubsystem spindexerSubsystem, IntakeSubsystem intakeSubsystem,
      IntakeRollerSubsystem intakeRollerSubsystem, DeflectorSubsystem deflectorSubsystem,
      TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    this.deflectorSubsystem = deflectorSubsystem;
    this.turretSubsystem = turretSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  public void generateTriggers() {
    new Trigger(() -> flywheelSubsystem.isJamDetected()).onTrue(unjamRoutineCommand());
  }

  private Command unjamRoutineCommand() {
    // TODO: tune
    final double reverseVolts = 4.0;
    final double reverseTime = 0.25;

    return flywheelSubsystem.stopCommand().alongWith(kickerSubsystem.stopCommand()).andThen(
        kickerSubsystem.runVoltageCommand(() -> -reverseVolts).withTimeout(reverseTime),
        kickerSubsystem.stopCommand());
  }

  public Command shootCommand() {
    return deflectorSubsystem.aimCommand().alongWith(
      flywheelSubsystem.aimCommand(),
      kickerSubsystem.runCommand(),
      new WaitUntilCommand(() -> kickerSubsystem.overMaxVelocity())
        .andThen(spindexerSubsystem.runCommand()));
  }

  public Command runIntakeCommand() {
    return intakeSubsystem.intakeDownCommand().alongWith(intakeRollerSubsystem.runCommand());
  }

  public Command ferryCommand() {
    return runIntakeCommand().alongWith(shootCommand());
  }
}
