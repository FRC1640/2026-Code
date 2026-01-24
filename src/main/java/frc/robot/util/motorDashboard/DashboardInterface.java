package frc.robot.util.motorDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public interface DashboardInterface {
  public abstract Command dashboardCommand(DoubleSupplier joystickValue);

  public abstract String getName();
}
