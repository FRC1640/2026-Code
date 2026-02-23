package frc.robot.util.control;

public class SinusoidalProfile {
  private double frequency;

  public static class Constraints {
    public final double maxVelocity;

    public Constraints(double maxVelocity) {
      this.maxVelocity = maxVelocity;
    }
  }

  public static class State {
    public double position;
    public double velocity;

    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }
  }

  private final Constraints constraints;

  public SinusoidalProfile(Constraints constraints) {
    this.constraints = constraints;
  }

  public State calculate(State current, double amplitude) {
    double omega = constraints.maxVelocity / amplitude;
    this.frequency = omega / (2 * Math.PI);
    double period = 1 / frequency;
    int segment = 0;
    if (current.position >= 0 && current.velocity <= 0) segment = 0;
    if (current.position <= 0 && current.velocity < 0) segment = 1;
    if (current.position <= 0 && current.velocity >= 0) segment = 2;
    if (current.position >= 0 && current.velocity > 0) segment = 3;
    double t = Math.asin(current.velocity / (-omega * amplitude)) / omega;
    if (t < 0) t += 2 * Math.PI;
    
  }
}
