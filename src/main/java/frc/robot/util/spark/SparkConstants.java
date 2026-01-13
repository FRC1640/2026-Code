package frc.robot.util.spark;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.drive.DriveConstants;

public class SparkConstants {
  public static final SparkConfiguration getGantryDefaultMax(int id) {
    return new SparkConfiguration(
        id,
        IdleMode.kBrake,
        true,
        60,
        50,
        16,
        StatusFrames.getDefault(),
        new LimitSwitchConfig()
            .reverseLimitSwitchEnabled(true)
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(false),
        new SparkMaxConfig());
  }

  public static final SparkConfiguration getLiftDefaultMax(int id, boolean inverted) {
    return new SparkConfiguration(
        id,
        IdleMode.kBrake,
        inverted,
        80,
        8,
        2,
        StatusFrames.getDefault(),
        new LimitSwitchConfig()
            .reverseLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            // .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchEnabled(false),
        new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultMax(int id, boolean inverted) {
    return new SparkConfiguration(
        id, IdleMode.kCoast, inverted, 60, 8, 2, StatusFrames.getDefault(), new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultMax(
      int id, boolean inverted, boolean useAbsolute) {
    return new SparkConfiguration(
        id,
        IdleMode.kCoast,
        inverted,
        60,
        8,
        2,
        StatusFrames.getDefault(),
        null,
        new LimitSwitchConfig(),
        new SparkMaxConfig(),
        useAbsolute);
  }

  public static final SparkConfiguration getDefaultMax(
      int id, boolean inverted, IdleMode breakMode) {
    return new SparkConfiguration(
        id, breakMode, inverted, 60, 8, 2, StatusFrames.getDefault(), new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultMaxIntake(
      int id, boolean inverted, IdleMode breakMode) {
    return new SparkConfiguration(
        id,
        breakMode,
        inverted,
        60,
        8,
        2,
        StatusFrames.getDefault(),
        new LimitSwitchConfig()
            .reverseLimitSwitchEnabled(true)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchType(Type.kNormallyOpen),
        new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultMax(
      int id, boolean inverted, boolean follower, SparkMax followerOf) {
    return new SparkConfiguration(
        id, IdleMode.kCoast, inverted, 60, 8, 2, StatusFrames.getDefault(), new SparkMaxConfig());
  }

  public static final SparkConfiguration getDefaultFlex(int id) {
    return new SparkConfiguration(
        id, IdleMode.kCoast, false, 45, 8, 2, StatusFrames.getDefault(), new SparkFlexConfig());
  }

  public static final SparkFlex driveFlex(int id) {
    return SparkConfigurer.configSparkFlex(
        new SparkConfiguration(
            id,
            IdleMode.kBrake,
            true,
            45,
            8,
            2,
            new StatusFrames(
                100, 20, (int) (1000 / DriveConstants.odometryFrequency), 500, 500, 500, 500),
            new SparkFlexConfig()));
  }

  public static final SparkConfiguration getClawMax(int id, boolean inverted, int smartCurrentLimit) {
    return new SparkConfiguration(id, IdleMode.kCoast, inverted, smartCurrentLimit, 8,
      2, StatusFrames.getDefault(), new SparkMaxConfig());
  }
}
