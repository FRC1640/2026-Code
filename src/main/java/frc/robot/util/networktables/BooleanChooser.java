package frc.robot.util.networktables;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BooleanChooser {
    String name;
    BooleanTopic topic;
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final BooleanEntry entry;
    public BooleanChooser(String name, boolean initValue){
        this.name = name;
        topic = inst.getBooleanTopic("/SmartDashboard/"+name);
        entry = topic.getEntry(initValue);
        entry.setDefault(initValue);
        entry.set(initValue);
    }
    public boolean getValue(){
        return entry.get();
    }

}