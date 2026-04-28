package frc.robot.subsystems.kicker;
import frc.robot.util.robotswitcher.SwitchableCANID;
public class KickerConstants {
  public static final int canId = SwitchableCANID.of(13).get();

  public static final double runVoltage = 8;
  public static final double runVelocityRPM = 5500;
  public static final double setpointToleranceRPM = 200;
  public static final double shooterToKickerVelocityConversion = 0.75;
}
