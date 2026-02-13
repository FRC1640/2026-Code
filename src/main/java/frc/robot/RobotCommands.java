package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.rollers.RollerConstants;
import frc.robot.subsystems.rollers.RollerSubsystem;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.deflector.DeflectorSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class RobotCommands {
  private final FlywheelSubsystem flywheelSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final RollerSubsystem rollerSubsystem;

  public RobotCommands(FlywheelSubsystem flywheelSubsystem, KickerSubsystem kickerSubsystem, SpindexerSubsystem spindexerSubsystem, IntakeSubsystem intakeSubsystem, RollerSubsystem rollerSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.rollerSubsystem = rollerSubsystem;
  }

  public void generateTriggers() {
    new Trigger(() -> flywheelSubsystem.isJamDetected()).onTrue(unjamRoutine());
  }

  private Command unjamRoutine() {
    //TODO: tune
    final double reverseVolts = 4.0;
    final double reverseTime = 0.25;

    return Commands.sequence(flywheelSubsystem.stopCommand(), kickerSubsystem.stopCommand(),
        kickerSubsystem.reverseVoltageCommand(reverseVolts).withTimeout(reverseTime),
        kickerSubsystem.stopCommand());
  }

  public Command shoot() {
    return kickerSubsystem.runCommand()
      .onlyWhile(()-> !kickerSubsystem.overMaxVelocity())
      .andThen(spindexerSubsystem.runCommand());
  }

  public Command runIntake(){
    return intakeSubsystem.intakeDownCommand().alongWith(rollerSubsystem.runCommand()).beforeStarting(()-> System.out.println("start")).finallyDo(()-> System.out.println("done"));
  }

  public Command ferryCommand(){
    return runIntake().alongWith(shoot());
  }
}
