package frc.robot.util.logging.PID;

import org.littletonrobotics.junction.Logger;

public class PIDLog {
  public static void log() {
    // PID Logging
    for (String id : PIDStorage.pidControllers.keySet()) {
      String folder = "PID/Regular/";
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/P", PIDStorage.getPID(id).getP());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/I", PIDStorage.getPID(id).getI());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/D", PIDStorage.getPID(id).getD());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/Period", PIDStorage.getPID(id).getPeriod());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/Setpoint", PIDStorage.getPID(id).getSetpoint());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/Accumulated Error",
          PIDStorage.getPID(id).getAccumulatedError());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/IZone", PIDStorage.getPID(id).getIZone());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/Error", PIDStorage.getPID(id).getError());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/Error Deriviative",
          PIDStorage.getPID(id).getErrorDerivative());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/Error Deriv Tolerance",
          PIDStorage.getPID(id).getErrorDerivativeTolerance());
      Logger.recordOutput(folder + PIDStorage.getPID(id) + "/Error Tolerance",
          PIDStorage.getPID(id).getErrorTolerance());
    }
  }
}
