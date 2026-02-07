package frc.robot.subsystems.intake;

import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class IntakeConstants {
  // TODO: change canids
  public static final int intakeCanID = SwitchableCANID.of(15).get();
  public static final int rollerCanID = SwitchableCANID.of(16).get();;
  public static final double gearRatio = 1;
  public static final double rollerGearRatio = 1;

  public static final Limits intakePositionLimits = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
      true); // TODO: change lims
}
