package frc.robot.util.control;

public class SinusoidalProfile {
  private double amplitude;
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

  public State calculate(double t, State current, State goal) {
    return new State(0, 0);
  }
}
