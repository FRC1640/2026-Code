package frc.robot.subsystems.shooter.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.limits.ExponentialMovingAverage;

public class FlywheelSubsystem extends SubsystemBase {
  private FlywheelIO io;
  private FlywheelIO.FlywheelIOInputs inputs = new FlywheelIO.FlywheelIOInputs();
  private ExponentialMovingAverage flywheelCurrentEMA;
  private boolean jamDetected = false;

  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
    flywheelCurrentEMA = new ExponentialMovingAverage(2.0, 10.0,
        () -> Math.max(inputs.flywheelMotorCurrent, inputs.flywheelMotorFollowerCurrent), "FlywheelCurrent");
  }
  public boolean isJamDetected() {
    return jamDetected;
  }
  public void clearJamDetected() {
    jamDetected = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Flywheel/currentMain", inputs.flywheelMotorCurrent);
    Logger.recordOutput("Flywheel/currentFollower", inputs.flywheelMotorFollowerCurrent);
    Logger.recordOutput("Flywheel/currentEMA", flywheelCurrentEMA.get());
    if (flywheelCurrentEMA.get() > 55.0) {
      jamDetected = true;
    }
  }
}
