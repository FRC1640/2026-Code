package frc.robot.util.spark;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.SpindexerConstants;

public class SparkConstants {
  public static final SparkFlexConfig shooterLeaderConfig;
  public static final SparkFlexConfig shooterFollowerConfig;
  public static final SparkMaxConfig hoodConfig;
  public static final SparkMaxConfig spindexerConfig;
  public static final SparkMaxConfig intakeConfig;
  public static final SparkMaxConfig intakeRollerConfig;
  public static final SparkMaxConfig kickerConfig;
  public static final SparkMaxConfig turretConfig;
  public static final SparkFlexConfig climberConfig;

  static {
    shooterLeaderConfig = getDefaultFlexConfig();
    shooterFollowerConfig = (SparkFlexConfig) getDefaultFlexConfig().follow(ShooterConstants.canId, true);
    hoodConfig = getDefaultMaxConfig();
    spindexerConfig = (SparkMaxConfig) getDefaultMaxConfig().inverted(SpindexerConstants.indexerSparkInverted);
    intakeConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true);
    intakeRollerConfig = getDefaultMaxConfig();
    kickerConfig = getDefaultMaxConfig();
    turretConfig = getDefaultMaxConfig();
    climberConfig = getDefaultFlexConfig();
  }

  private static final SparkMaxConfig getDefaultMaxConfig() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast).inverted(false);
    return config;
  }

  private static final SparkFlexConfig getDefaultFlexConfig() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast).inverted(false);
    return config;
  }

  public static final SparkConfiguration getDefaultMax(int id, boolean inverted) {
    return new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2, StatusFrames.getDefault(),
        new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultMax(int id, boolean inverted, boolean useAbsolute) {
    return new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2, StatusFrames.getDefault(), null,
        new LimitSwitchConfig(), new SparkMaxConfig(), useAbsolute);
  }

  public static final SparkConfiguration getDefaultMax(int id, boolean inverted, IdleMode brakeMode) {
    return new SparkConfiguration(id, brakeMode, inverted, 60, 8, 2, StatusFrames.getDefault(),
        new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultMax(int id, boolean inverted, SparkMax followerOf) {
    SparkConfiguration sc = new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2,
        StatusFrames.getDefault(), new SparkMaxConfig());
    sc.follow(followerOf);
    return sc;
  }

  public static final SparkConfiguration getDefaultFlex(int id) {
    return new SparkConfiguration(id, IdleMode.kCoast, false, 45, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }

  public static final SparkConfiguration getDefaultFlex(int id, boolean inverted) {
    return new SparkConfiguration(id, IdleMode.kCoast, inverted, 45, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }

  public static final SparkConfiguration getDefaultFlex(int id, boolean inverted, SparkFlex followerOf) {
    SparkConfiguration sc = new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2,
        StatusFrames.getDefault(), new SparkFlexConfig());
    sc.follow(followerOf);
    return sc;
  }

  public static final SparkConfiguration getShooterFlex(int id) {
    return new SparkConfiguration(id, IdleMode.kCoast, false, 80, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }
  public static final SparkConfiguration getShooterFlex(int id, boolean inverted) {
    return new SparkConfiguration(id, IdleMode.kCoast, inverted, 80, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }

  public static final SparkConfiguration getShooterFlex(int id, boolean inverted, SparkFlex followerOf) {
    SparkConfiguration sc = new SparkConfiguration(id, IdleMode.kCoast, inverted, 80, 8, 2,
        StatusFrames.getDefault(), new SparkFlexConfig());
    sc.follow(followerOf, true);
    return sc;
  }
  public static final SparkFlex driveFlex(int id) {
    return SparkConfigurer.configSparkFlex(new SparkConfiguration(id, IdleMode.kBrake, true, 45, 8, 2,
        new StatusFrames(100, 20, (int) (1000 / DriveConstants.odometryFrequency), 500, 500, 500, 500),
        new SparkFlexConfig()));
  }
}
