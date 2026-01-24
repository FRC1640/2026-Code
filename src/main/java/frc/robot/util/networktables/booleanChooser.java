package frc.robot.util.networktables;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BooleanChooser {
    private String name;
    private BooleanTopic topic;
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final BooleanEntry entry;

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
