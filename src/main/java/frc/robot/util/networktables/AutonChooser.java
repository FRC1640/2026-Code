package frc.robot.util.networktables;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonChooser {
    SendableChooser<String> dropdown = new SendableChooser<String>();
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    String name;
    StringTopic topic;
    StringEntry entry;
    
    public AutonChooser(String name, String[] autons){
        dropdown = new SendableChooser<String>();
        this.name = name;
        for (String i : autons){
            dropdown.addOption(i, i);
        }
        SmartDashboard.putData(this.name, dropdown);
        topic = inst.getStringTopic("/SmartDashboard/"+name+"/selected");
        entry = topic.getEntry(null);
    }

    public Command getAuto(){
        return AutoBuilder.buildAuto(entry.get());
    }
    public String getString(){
        return entry.get();
    }

}
