package frc.robot.subsystems.kicker;
import frc.robot.util.robotswitcher.SwitchableCANID;
public class KickerConstants {
  public static final int canId = SwitchableCANID.of(13).get();

  // TODO: tune
  public static final double runVoltage = 8;
  public static final double runVelocityRPM = 3800;
  public static final double setpointToleranceRPM = 50;
}
