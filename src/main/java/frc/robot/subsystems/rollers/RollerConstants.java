package frc.robot.subsystems.rollers;

import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class RollerConstants {
  // TODO: change canids
  public static final int canID = SwitchableCANID.of(16).get();;
  public static final double gearRatio = 1;

  public static final double runVoltage = 8; //TODO: change

  public static final Limits intakePositionLimits = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
      true); // TODO: change lims

}
