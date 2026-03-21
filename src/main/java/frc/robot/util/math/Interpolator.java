package frc.robot.util.math;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Interpolator {
  private InterpolatingDoubleTreeMap distanceToHoodAngle = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap distanceToShooterVelocity = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap distanceToTimeOfFlight = new InterpolatingDoubleTreeMap();

  public Interpolator() {

  }

  public void put(double distance, double hoodAngle, double shooterVelocity, double timeOfFlight) {
    distanceToHoodAngle.put(distance, hoodAngle);
    distanceToShooterVelocity.put(distance, shooterVelocity);
    distanceToTimeOfFlight.put(distance, timeOfFlight);
  }
  public void put(double distance, double hoodAngle, double shooterVelocity) {
    distanceToHoodAngle.put(distance, hoodAngle);
    distanceToShooterVelocity.put(distance, shooterVelocity);
  }

  public void putTime(double distance, double timeOfFlight) {
    distanceToTimeOfFlight.put(distance, timeOfFlight);
  }

  public double getHoodAngle(double distance) {
    return distanceToHoodAngle.get(distance);
  }

  public double getShooterVelocity(double distance) {
    return distanceToShooterVelocity.get(distance);
  }

  public double getTimeOfFlight(double distance) {
    return distanceToTimeOfFlight.get(distance);
  }
}
