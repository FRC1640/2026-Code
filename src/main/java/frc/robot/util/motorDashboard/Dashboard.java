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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Dashboard {
  private static SendableChooser<String> dropdown = new SendableChooser<String>();
  private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static Dashboard instance;
  private static HashMap<String, DashboardInterface> subsystemHashmap = new HashMap<>();

  private static StringTopic topic;
  private static StringEntry entry;

  private CommandXboxController dashboardController;
  
  public Dashboard(List<DashboardInterface> dashboardInterfaceList) {
    instance = this;
    for (DashboardInterface dashboardInterface : dashboardInterfaceList) {
      dropdown.addOption(dashboardInterface.getName(), dashboardInterface.getName());
      subsystemHashmap.put(dashboardInterface.getName(), dashboardInterface);
    }
    SmartDashboard.putData("DashboardDropdown", dropdown);
    topic = inst.getStringTopic("/SmartDashboard/DashboardDropdown/selected");
    entry = topic.getEntry(null);
    dashboardController = new CommandXboxController(2);
    new Trigger(() -> Math.abs(dashboardController.getLeftTriggerAxis()) > 0.03).whileTrue(Dashboard.dashboardCommand(() -> dashboardController.getLeftY()));
  }
  public static Dashboard getInstance() {
    return instance;
  }

  public static Command dashboardCommand(DoubleSupplier joystickValue) {
    return subsystemHashmap.get(entry.get()).dashboardCommand(joystickValue);
  }

}
