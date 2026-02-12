package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

public class RobotCommands {
  private final FlywheelSubsystem flywheelSubsystem;
  private final KickerSubsystem kickerSubsystem;

  public RobotCommands(FlywheelSubsystem flywheelSubsystem, KickerSubsystem kickerSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    this.kickerSubsystem = kickerSubsystem;
  }

  public void generateTriggers() {
    new Trigger(() -> flywheelSubsystem.isJamDetected()).onTrue(unjamRoutine());
  }

  private Command unjamRoutine() {
    final double reverseVolts = 4.0; // tune these 2
    final double reverseTime = 0.25;

    return Commands.sequence(flywheelSubsystem.stopCommand(), kickerSubsystem.stopCommand(),
        kickerSubsystem.runVoltageCommand(() -> -reverseVolts).withTimeout(reverseTime),
        kickerSubsystem.stopCommand());
  }
}
