package frc.robot.util.motorDashboard;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class Dashboard {
    private static Dashboard instance;
    private static HashMap<String, DashboardInterface> subystemHashmap;
    private static List<DashboardInterface> dashboardInterfaceList;
    private static DashboardInterface currentSubsystem;
    public Dashboard(List<DashboardInterface> dashboardInterfaceList){
        this.dashboardInterfaceList = dashboardInterfaceList;
        instance = this;
        for (DashboardInterface dashboardInterface : dashboardInterfaceList) {
            subystemHashmap.put(dashboardInterface.getName(), dashboardInterface);
        }
    }
    public static Dashboard getInstance(){
        return instance;
    }

    public static Command dashboardCommand() {
        return currentSubsystem.dashboardCommand();
    }
}
