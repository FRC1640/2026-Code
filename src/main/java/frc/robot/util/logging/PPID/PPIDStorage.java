package frc.robot.util.logging.PPID;

import java.util.HashMap;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.util.logging.ConsoleColors;

public class PPIDStorage {
    private static int pidAmount = 0;
    public static HashMap<String, ProfiledPIDController> pidControllers = new HashMap<>();

    public static void addPID(String name, ProfiledPIDController j) {
        pidAmount++;

        pidControllers.put(name, j);
    }

    public static void addPID(ProfiledPIDController j) {
        addPID("PID-" + pidAmount, j);

    }

    public static ProfiledPIDController getPID(String name) {
        if (pidControllers.get(name) == null) {
            ConsoleColors.colorize("ERROR: PID with name " + name + " does not exist.", ConsoleColors.RED);
        }
        return pidControllers.get(name);
    }
}
