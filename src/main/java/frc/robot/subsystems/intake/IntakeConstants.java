package frc.robot.subsystems.intake;

import frc.robot.util.limits.Limits;

public class IntakeConstants {
  //TODO: change canids
  public static final int canID = -1;
  public static final int rollerCanID = -1;
  public static final double gearRatio = 1;
  public static final double rollerGearRatio = 1;

  public static final Limits intakePositionLimits = new Limits(0.5, 0.9, true); // TODO: change lims
}
