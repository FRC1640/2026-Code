package frc.robot.subsystems.shooter;

import frc.robot.util.robotswitcher.SwitchableCANID;

public class ShooterConstants {

  public static final int canId = SwitchableCANID.of(10).get();
  public static final int followerCanId = SwitchableCANID.of(11).get();
  public static final double jamCurrentAmps = 55.0;
  public static final double setpointVelocityToleranceRPM = 350;
}
