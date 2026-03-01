package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class ClimberConstants {
  public static final int canId = SwitchableCANID.of(17).get();
  public static final double climberGearRatioSim = 1.0;
  public static final Limits positionLimitsMeters = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY,
      true);
  public static final double climberAngleRadians = Units.degreesToRadians(5); // TODO
  public static final double climberRetractedHeight = 0; // TODO

  public static final double encoderOffset = 0;

  public static final double climberEncoderCount0 = 0 + encoderOffset;

  public static final double climberEncoderCount1 = 0.745 + encoderOffset;
  
  public static final double climberHeightCount0 = 14 / 100;

  public static final double climberHeightCount1 = 35 / 100
  ;

  // custom formatting
  /** Conversion factor from encoder counts to radian angle. */
  public static final double climberRatio =
      (climberHeightCount1 - climberHeightCount0
      )
      / (climberEncoderCount1 - climberEncoderCount0);
}
