package frc.robot.util.motorDashboard;

import edu.wpi.first.wpilibj2.command.Command;

public class Dashboard {
    private static Dashboard instance;
    public Dashboard(){
        instance = this;
    }
    public static Dashboard getInstance(){
        return instance;
    }

    public static Command dashboardCommand(DashboardInterface subsystem){
        return subsystem.dashboardCommand();
    }
}
