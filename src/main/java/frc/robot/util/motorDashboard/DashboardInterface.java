package frc.robot.util.motorDashboard;

import edu.wpi.first.wpilibj2.command.Command;

public interface DashboardInterface {
    public abstract Command dashboardCommand();

    public abstract String getName();    
}
