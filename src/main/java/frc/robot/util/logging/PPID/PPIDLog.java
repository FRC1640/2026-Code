package frc.robot.util.logging.PPID;

import org.littletonrobotics.junction.Logger;

public class PPIDLog {
    public static void log() {
        // PID Logging
        for (String id : PPIDStorage.pidControllers.keySet()) {
            String folder = "PID/Profiled/";
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/P", PPIDStorage.getPID(id).getP());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/I", PPIDStorage.getPID(id).getI());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/D", PPIDStorage.getPID(id).getD());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Period",
                    PPIDStorage.getPID(id).getPeriod());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Goal/Position",
                    PPIDStorage.getPID(id).getGoal().position);
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Goal/Velocity",
                    PPIDStorage.getPID(id).getGoal().velocity);
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Setpoint/Position",
                    PPIDStorage.getPID(id).getSetpoint().position);
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Setpoint/Velocity",
                    PPIDStorage.getPID(id).getSetpoint().velocity);
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Accumulated Error",
                    PPIDStorage.getPID(id).getAccumulatedError());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/IZone",
                    PPIDStorage.getPID(id).getIZone());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Position Error",
                    PPIDStorage.getPID(id).getPositionError());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Position Tolerance",
                    PPIDStorage.getPID(id).getPositionTolerance());
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Constraints/Max Acceleration",
                    PPIDStorage.getPID(id).getConstraints().maxAcceleration);
            Logger.recordOutput(folder + PPIDStorage.getPID(id) + "/Constraints/Max Velocity",
                    PPIDStorage.getPID(id).getConstraints().maxVelocity);

        }
    }
}
