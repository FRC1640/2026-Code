package frc.robot.util.networktables;

import java.security.KeyStore.Entry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DropdownChooser <T> {
    SendableChooser<T> dropdown = new SendableChooser<T>();
    StringTopic topic;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    StringEntry entry;
    public DropdownChooser(String name, SendableChooser<T> chooser, String defaultOpt){
        SmartDashboard.putData(name, chooser);
        topic = inst.getStringTopic("/SmartDashboard/"+name);
        // entry = topic.getEntry();
    }
    // public T getData(){

    // }
}
