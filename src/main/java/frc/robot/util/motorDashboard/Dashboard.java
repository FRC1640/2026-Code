package frc.robot.util.motorDashboard;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class Dashboard {
  private static SendableChooser<String> dropdown = new SendableChooser<String>();
  private static Dashboard instance;
  private static HashMap<String, SubsystemPlatform> subsystemHashmap = new HashMap<>();

  private static CommandXboxController dashboardController;

  public Dashboard(SubsystemPlatform... subsystems) {
    instance = this;
    for (SubsystemPlatform subsystem : subsystems) {
      dropdown.addOption(subsystem.getName(), subsystem.getName());
      subsystemHashmap.put(subsystem.getName(), subsystem);
    }
    SmartDashboard.putData("DashboardDropdown", dropdown);
    dashboardController = new CommandXboxController(2);
    new Trigger(() -> (Math.abs(dashboardController.getLeftY()) > 0.03
        || Math.abs(dashboardController.getRightY()) > 0.03)).whileTrue(
            executeCommand(() -> dashboardController.getLeftY(), () -> dashboardController.getRightY()));
  }
  public static Dashboard getInstance() {
    return instance;
  }

  public static Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return subsystemHashmap.get(dropdown.getSelected()).dashboardCommand(leftJoystickValue, rightJoystickValue);
  }

  public static Command executeCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    Command c = new Command() {
      Command internal;

      @Override
      public void initialize() {
        if (dropdown.getSelected() != null) {
          internal = dashboardCommand(leftJoystickValue, rightJoystickValue);
        }
        CommandScheduler.getInstance().schedule(internal);
      }

      @Override
      public void end(boolean interrupted) {
        internal.cancel();
      }
      @Override
      public boolean isFinished() {
        return Math.abs(dashboardController.getLeftY()) < 0.03
            && Math.abs(dashboardController.getRightY()) < 0.03;
      }
    };
    return c;

  }
}
