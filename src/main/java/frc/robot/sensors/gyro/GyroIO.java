package frc.robot.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean isConnected = false;
    public boolean isCalibrating = false;
    public double angleRadiansRaw = 0.0;
    public double angularVelocityDegreesPerSecond = 0.0;
    public double angleDegreesRaw = 0.0;
    public double displacementX = 0.0;
    public double displacementY = 0.0;

    public Rotation2d roll = new Rotation2d();
    public Rotation2d pitch = new Rotation2d();

    public double accelX;
    public double accelY;
    public double accelZ;

    public double[] odometryYawTimestamps = new double[]{};
    public Rotation2d[] odometryYawPositions = new Rotation2d[]{};
    public double[] odometryYawRate = new double[]{};
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }

  public default double getActual(GyroIOInputs inputs) {
    return 0;
  }

  public default double getOffset() {
    return 0;
  }

  public default void setOffset(double offset) {
  }

  public static GyroIO getIOByMode(DoubleSupplier simRotRate) {
    return switch (Robot.getMode()) {
      case REAL -> new GyroIONavX();
      case SIM -> new GyroIOSim(simRotRate, COTS.ofNav2X().get());
      case REPLAY -> new GyroIO() {
      };
    };
  }
}
