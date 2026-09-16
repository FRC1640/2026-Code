package frc.robot.sensors.gyro;

import java.util.Queue;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.sensors.odometry.SparkOdometryThread;

public class GyroIOCanandgyro implements GyroIO {
  private final Canandgyro gyro;

  double offset = 0;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> rate;

  double pitchOffset = 0;
  double rollOffset = 0;

  public GyroIOCanandgyro(int gyroId) {
    gyro = new Canandgyro(gyroId);
    rate = SparkOdometryThread.getInstance().registerSignal(() -> Math.toRadians(gyro.getAngularVelocityYaw() * 360));
    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> gyro.getYaw() * 2 * Math.PI);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {

    inputs.isConnected = gyro.isConnected();
    inputs.isCalibrating = gyro.isCalibrating();
    inputs.angleRadiansRaw = gyro.getYaw() * 2 * Math.PI;
    inputs.angularVelocityDegreesPerSecond = gyro.getAngularVelocityYaw() * 360;
    inputs.angleDegreesRaw = Math.toDegrees(inputs.angleRadiansRaw);

    inputs.roll = new Rotation2d(gyro.getRoll() * 2 * Math.PI - rollOffset);
    inputs.pitch = new Rotation2d(gyro.getPitch() * 2 * Math.PI - pitchOffset);

    inputs.displacementX = 0;
    inputs.displacementY = 0;
    inputs.odometryYawRate = rate.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream().map((Double value) -> Rotation2d.fromRadians(value))
        .toArray(Rotation2d[]::new);

    inputs.accelX = gyro.getAccelerationX();
    inputs.accelY = gyro.getAccelerationY();
    inputs.accelZ = gyro.getAccelerationZ();
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    rate.clear();
  }

  @Override
  public double getActual(GyroIOInputs inputs) {
    return inputs.angleRadiansRaw - offset;
  }

  @Override
  public double getOffset() {
    return offset;
  }

  @Override
  public void setOffset(double offset) {
    this.offset = offset;
  }
}
