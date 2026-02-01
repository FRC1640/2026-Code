package frc.robot.util.robotswitcher;

import java.util.HashMap;

import frc.robot.constants.RobotConstants;

public class SwitchableCANID {

    HashMap<String, Integer> ids = new HashMap<>();
    int defID;

    public SwitchableCANID(int defID) {
        this.defID = defID;
    }

    public SwitchableCANID addAlt(RobotType robot, int id) {
        ids.put(robot.getName(), id);
        return this;
    }

    public static SwitchableCANID of(int defID) {
        return new SwitchableCANID(defID);
    }

    public int get() {
        return ids.getOrDefault(RobotConstants.RobotInformation.robot.getName(), defID);
    }
}
