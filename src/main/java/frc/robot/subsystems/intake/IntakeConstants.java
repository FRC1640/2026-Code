package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.util.limits.Limits;
import frc.robot.util.robotswitcher.SwitchableCANID;

public class IntakeConstants {
  /** Intake CAN ID. */
  public static final int canId = SwitchableCANID.of(15).get();

  /** Offset from encoder zero position to the vertical. */
  public static final double intakeZeroOffsetRadians = Units.degreesToRadians(0); // TODO

  /** Intaking (down) position with the vertical. */
  public static final double activePositionRadians = Units.degreesToRadians(0); // TODO
  /** Stowed (up) position with the vertical. */
  public static final double stowedPositionRadians = Units.degreesToRadians(90); // TODO

  /** Intake position limits in radians. */
  public static final Limits positionLimits = new Limits(0, Math.PI / 2, true); // TODO
}
