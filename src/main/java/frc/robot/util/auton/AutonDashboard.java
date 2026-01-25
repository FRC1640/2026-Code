package frc.robot.util.auton;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConstants.Autons;

public class AutonDashboard {
    private SendableChooser<String> dropdown = new SendableChooser<String>();
    
    public AutonDashboard(){
        autonInit();
    }

    private void autonInit(){
        dropdown.setDefaultOption("None", "None");
        for (String auton : Autons.autonNames){
            dropdown.addOption(auton, auton);
        }
        SmartDashboard.putData("Choose Auton", dropdown);
    }

    public Command getAuto(){
        return AutoBuilder.buildAuto(dropdown.getSelected());
    }

    public String getString(){
        return dropdown.getSelected();
    }

}
