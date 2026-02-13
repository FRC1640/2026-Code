package frc.robot.subsystems.intake;

import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class IntakeConstants {
  // TODO: change canids
  public static final int canID = SwitchableCANID.of(15).get();
  public static final double gearRatio = 1;

  public static final double downPosition = 5; // TODO: change
  public static final double upPosition = 10; // TODO: change

  public static final Limits positionLimits = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true); // TODO:
  // change
  // lims
}
