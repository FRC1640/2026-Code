package frc.robot.util.networktables;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SysIdChooser {
    private SendableChooser<Command> dropdown = new SendableChooser<Command>();
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private String name;

    public SysIdChooser(String name, ArrayList<Command> SysIdRoutines) {
        dropdown = new SendableChooser<Command>();
        this.name = name;
        for (Command i : SysIdRoutines) {
            dropdown.addOption(i.getName(), i);
        }
        SmartDashboard.putData(this.name, dropdown);
    }

    public Command getSysIdRoutine() {
        return dropdown.getSelected();
    }

    public String getString() {
        return dropdown.getSelected().getName();
    }

}
