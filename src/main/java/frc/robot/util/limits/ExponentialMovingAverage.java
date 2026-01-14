package frc.robot.util.limits;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.periodic.PeriodicBase;

public class ExponentialMovingAverage extends PeriodicBase {
  // https://www.investopedia.com/terms/e/ema.asp

  private DoubleSupplier dataSupplier;
  private double multiplier; // input for name, supplier for data
  private double current;
  private double smoothing;
  private double period;

  private String name;

  public ExponentialMovingAverage(double smoothing, double period, DoubleSupplier dataSupplier) {
    this.multiplier = smoothing / (1 + period);
    this.smoothing = smoothing;
    this.period = period;
    this.dataSupplier = dataSupplier;
  }

  public ExponentialMovingAverage(double smoothing, double period, DoubleSupplier dataSupplier, String name) {
    this.multiplier = smoothing / (1 + period);
    this.dataSupplier = dataSupplier;
    this.smoothing = smoothing;
    this.period = period;
    this.name = name;
  }

  public double get() {
    return current;
  }

  @Override
  public void periodic() {
    current = multiplier * dataSupplier.getAsDouble() + (multiplier * current);
    if (name != null) {
      Logger.recordOutput("EMA/" + name + "/Smoothing", smoothing);
      Logger.recordOutput("EMA/" + name + "/Period", period);
      Logger.recordOutput("EMA/" + name + "/Multiplier", multiplier);
      Logger.recordOutput("EMA/" + name + "/Current", current);
    }
  }
}
