package frc.robot.subsystems.climber;

import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class ClimberConstants {
  public static final int canId = SwitchableCANID.of(17).get();
  public static final Limits limits = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true);
  public static final double climberGearRatioSim = 1.0;
}
