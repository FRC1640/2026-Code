package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  public void configureBindings() {

  }
  
  public Command getAutonomousCommand() {
    return new PrintCommand("No autonomous command configured.");
  }
}
