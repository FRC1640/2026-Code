package frc.robot.util.motorDashboard;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;

public class Dashboard {
    private static Dashboard instance;
    private static List<DashboardInterface> interfaces;
    public Dashboard(List<DashboardInterface> interfaces){
        instance = this;
        this.interfaces = interfaces;
    }
    public static Dashboard getInstance(){
        return instance;
    }

    
}
