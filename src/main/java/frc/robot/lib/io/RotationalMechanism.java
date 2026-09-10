package frc.robot.lib.io;

import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;

public class RotationalMechanism {

  public final String logPath;
  private DoubleConsumer setVelocityRadPerSec;

  public RotationalMechanism(String logPath, DoubleConsumer setVelocityRadPerSec) {
    this.logPath = logPath + (logPath.endsWith("/") ? "" : "/");
    this.setVelocityRadPerSec = setVelocityRadPerSec;
  }

  public final void setVelocityRadPerSec(double velocityRadPerSec) {
    Logger.recordOutput(logPath + "setpointVelocityRadPerSec", velocityRadPerSec);
    Logger.recordOutput(logPath + "setpointVelocityRPM", velocityRadPerSec * 60 / (2 * Math.PI));
    setVelocityRadPerSec.accept(velocityRadPerSec);
  }
}
