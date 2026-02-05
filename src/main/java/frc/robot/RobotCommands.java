package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;

public class RobotCommands {
  private final FlywheelSubsystem flywheelSubsystem;
  private final HopperSubsystem hopperSubsystem;

  public RobotCommands(FlywheelSubsystem flywheelSubsystem, HopperSubsystem hopperSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    this.hopperSubsystem = hopperSubsystem;
  }

  public void generateTriggers() {
    new Trigger(() -> flywheelSubsystem.isJamDetected())
        .onTrue(unjamRoutine());
  }

  private Command unjamRoutine() {
    final double reverseVolts = 4.0;   // tune these 2 
    final double reverseTime = 0.25;   

    return Commands.sequence(
        flywheelSubsystem.stopCommand(),                       
        hopperSubsystem.stopCommand(),                       
        hopperSubsystem.reverseVoltageCommand(reverseVolts)   
            .withTimeout(reverseTime),
        hopperSubsystem.stopCommand()                          
    );
  }
}
