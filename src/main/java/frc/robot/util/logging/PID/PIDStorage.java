package frc.robot.util.logging.PID;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.logging.ConsoleColors;

public class PIDStorage {
    private static int pidAmount = 0;
    public static HashMap<String, PIDController> pidControllers = new HashMap<>();

    public static void addPID(String name, PIDController j) {
        pidAmount++;

        pidControllers.put(name, j);
    }

    public static void addPID(PIDController j) {
        addPID("PID-" + pidAmount, j);
            
    }

    
    public static PIDController getPID(String name) {
        if(pidControllers.get(name) == null){
            ConsoleColors.colorize("PIDController with name " + name + " does not exist!" + ConsoleColors.RESET, ConsoleColors.RED);
            return null;
        } else {
            return pidControllers.get(name);
        }
    }
}
