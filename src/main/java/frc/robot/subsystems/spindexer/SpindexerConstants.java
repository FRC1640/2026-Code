package frc.robot.subsystems.spindexer;

import frc.robot.util.robotswitcher.SwitchableCANID;

public class SpindexerConstants {

  public static final int indexerSparkCanId = SwitchableCANID.of(14).get();

  public static final boolean indexerSparkInverted = false;

  public static final double indexerGearRatioSim = 1;

  public static final double runVoltage = 8;
  public static final double runVelocity = 3300; // 3150 rpm
  public static final double jamCurrentThresh = 60;
}
