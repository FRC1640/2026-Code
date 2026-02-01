package frc.robot.util.spark;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.drive.DriveConstants;

public class SparkConstants {
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

  public static final SparkConfiguration getFlywheelFlex(int id) {
    return new SparkConfiguration(id, IdleMode.kCoast, false, 60, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }
  public static final SparkConfiguration getFlywheelFlex(int id, boolean inverted) {
    return new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }

  public static final SparkConfiguration getFlywheelFlex(int id, boolean inverted, SparkFlex followerOf) {
    SparkConfiguration sc = new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2,
        StatusFrames.getDefault(), new SparkFlexConfig());
    sc.follow(followerOf);
    return sc;
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

  public static final SparkConfiguration getFlywheelFlex(int id) {
    return new SparkConfiguration(id, IdleMode.kCoast, false, 60, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }
  public static final SparkConfiguration getFlywheelFlex(int id, boolean inverted) {
    return new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2, StatusFrames.getDefault(),
        new SparkFlexConfig());
  }

  public static final SparkConfiguration getFlywheelFlex(int id, boolean inverted, SparkFlex followerOf) {
    SparkConfiguration sc = new SparkConfiguration(id, IdleMode.kCoast, inverted, 60, 8, 2,
        StatusFrames.getDefault(), new SparkFlexConfig());
    sc.follow(followerOf);
    return sc;
  }
  public static final SparkFlex driveFlex(int id) {
    return SparkConfigurer.configSparkFlex(new SparkConfiguration(id, IdleMode.kBrake, true, 45, 8, 2,
        new StatusFrames(100, 20, (int) (1000 / DriveConstants.odometryFrequency), 500, 500, 500, 500),
        new SparkFlexConfig()));
  }
}
