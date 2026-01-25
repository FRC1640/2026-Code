package frc.robot.util.motorDashboard;

import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Dashboard {
  private static SendableChooser<String> dropdown = new SendableChooser<String>();
  private static Dashboard instance;
  private static HashMap<String, DashboardInterface> subsystemHashmap = new HashMap<>();

  private CommandXboxController dashboardController;
  
  public Dashboard(DashboardInterface... interfaces) {
    instance = this;
    for (DashboardInterface dashboardInterface : interfaces) {
      dropdown.addOption(dashboardInterface.getName(), dashboardInterface.getName());
      subsystemHashmap.put(dashboardInterface.getName(), dashboardInterface);
    }
    SmartDashboard.putData("DashboardDropdown", dropdown);
    dashboardController = new CommandXboxController(2);
    new Trigger(() -> Math.abs(dashboardController.getLeftY()) > 0.03).whileTrue(executeCommand(() -> dashboardController.getLeftY()));
  }
  public static Dashboard getInstance() {
    return instance;
  }

  public static Command dashboardCommand(DoubleSupplier joystickValue) {
    return subsystemHashmap.get(dropdown.getSelected()).dashboardCommand(joystickValue);
  }
  
  public static Command executeCommand(DoubleSupplier joystickValue) {
    Command c = new Command() {
      Command internal;

      @Override
      public void initialize() {
        if (dropdown.getSelected() != null){
        internal = dashboardCommand(joystickValue);
        }
        internal.schedule();
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
    return c;

  }
}
