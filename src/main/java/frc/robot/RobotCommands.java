package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterControl;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class RobotCommands {
  private final FlywheelSubsystem flywheelSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private final SpindexerSubsystem spindexerSubsystem;

  public RobotCommands(FlywheelSubsystem flywheelSubsystem, KickerSubsystem kickerSubsystem, SpindexerSubsystem spindexerSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.spindexerSubsystem = spindexerSubsystem;
  }

  public void generateTriggers() {
    new Trigger(() -> flywheelSubsystem.isJamDetected()).onTrue(unjamRoutineCommand());
  }

  private Command unjamRoutineCommand() {
    final double reverseVolts = 4.0; // tune these 2
    final double reverseTime = 0.25;

    return flywheelSubsystem.stopCommand().alongWith(kickerSubsystem.stopCommand()).andThen(
        kickerSubsystem.runVoltageCommand(() -> -reverseVolts).withTimeout(reverseTime),
        kickerSubsystem.stopCommand());
  }

  // SHOOTER CONTROL COMMANDS
  public Command shootCommand() {
    ShooterControl shooterControl = ShooterControl.getInstance();
    return flywheelSubsystem.runVelocityCommand()
        .alongWith(new InstantCommand(() -> shooterControl.setShooting(true)),
            (new WaitUntilCommand(() -> flywheelSubsystem.isAtSetpoint()))
                .andThen(kickerSubsystem.runVoltageCommand(() -> 6.0)).alongWith(spindexerSubsystem.runVoltageCommand(() -> 6.0)))
        .finallyDo(() -> {
          shooterControl.setShooting(false);
        });
  }
}
