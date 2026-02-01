package frc.robot.util.limits;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.periodic.PeriodicBase;

public class SimpleMovingAverage extends PeriodicBase {
  private String logName;
  private DoubleSupplier data;
  private ArrayList<Double> numList;
  private int period;

  public SimpleMovingAverage(int period, DoubleSupplier data, String logName) {
    numList = new ArrayList<>();
    this.period = period;
    this.data = data;
    this.logName = logName;
  }

  public void periodic() {
    numList.add(data.getAsDouble());
    if (numList.size() > period) {
      numList.remove(0);
    }
    Logger.recordOutput(logName, get());
  }

  public double get() {
    double total = 0;
    for (double data : numList) {
      total += data;
    }
    return total / numList.size();
  }
}
