package frc.robot.util.driveTesting;

import java.lang.module.ModuleDescriptor.Builder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.motorDashboard.Dashboard;

public class DriveTestDashboard {
    Sendable velocitySlider = new Sendable() {

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("Number Slider");
            builder.addDoubleProperty("min_value", ()-> 0, null);
            builder.addDoubleProperty("max_value", ()-> 5, null);
            builder.addBooleanProperty("publish_all", ()-> true, null);
        }
        
    };
    private static DriveTestDashboard instance;
    private static double velocity;

    public DriveTestDashboard(){
        SmartDashboard.putData("Velocity", velocitySlider);
        instance = this;
        
    }

    public static DriveTestDashboard getInstance(){
        return instance;
    }
}
