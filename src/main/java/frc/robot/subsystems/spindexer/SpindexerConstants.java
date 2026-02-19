package frc.robot.subsystems.spindexer;

import frc.robot.util.robotswitcher.SwitchableCANID;

public class SpindexerConstants {

  public static final int indexerSparkCanId = SwitchableCANID.of(14).get();; // TODO: set id and inverted and set to actual
  // values
  public static final boolean indexerSparkInverted = false;

  public static final double indexerGearRatioSim = 1;

  public static final double runVoltage = 1;
}
