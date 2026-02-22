package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class ClimberConstants {
  public static final int canId = SwitchableCANID.of(17).get();
  public static final double climberGearRatioSim = 1.0;
  public static final Limits positionLimitsMeters = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
      true);
  public static final double climberAngleRadians = Units.degreesToRadians(5); // TODO
  public static final double climberRetractedHeight = 0; // TODO
}
