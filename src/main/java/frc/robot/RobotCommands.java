package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

public class RobotCommands {
  private final FlywheelSubsystem flywheel;
  private final HopperSubsystem hopper;

  public RobotCommands(FlywheelSubsystem flywheel, HopperSubsystem hopper) {
    this.flywheel = flywheel;
    this.hopper = hopper;
  }

  public void generateTriggers() {
    new Trigger(() -> flywheel.isJamDetected())
        .onTrue(unjamRoutine());
  }
  private Command unjamRoutine() {
    final double reverseVolts = 4.0;  // tune
    final double reverseTime = 0.25;  // tune
    return Commands.sequence(
        Commands.runOnce(() -> hopper.stop()),
        hopper.reverseVoltageCommand(reverseVolts).withTimeout(reverseTime),
        Commands.runOnce(() -> hopper.stop()),
        Commands.runOnce(() -> flywheel.clearJamDetected())
    );
  }
}
