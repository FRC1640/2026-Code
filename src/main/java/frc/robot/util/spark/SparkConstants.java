package frc.robot.util.spark;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.RobotConstants.RobotTypes;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.util.robotswitcher.Switchable;

public class SparkConstants {
  public static final SparkFlexConfig driveConfig;
  public static final SparkMaxConfig steerConfig;
  public static final SparkFlexConfig shooterLeaderConfig;
  public static final SparkFlexConfig shooterFollowerConfig;
  public static final SparkMaxConfig hoodConfig;
  public static final SparkMaxConfig spindexerConfig;
  public static final SparkMaxConfig intakeConfig;
  public static final SparkMaxConfig intakeRollerConfig;
  public static final SparkFlexConfig kickerConfig;
  public static final SparkMaxConfig turretConfig;
  public static final SparkFlexConfig climberConfig;

  static {
    driveConfig = getDefaultFlexConfig();
    driveConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(45).encoder
        .quadratureMeasurementPeriod(8).quadratureAverageDepth(2);
    // configure report rate for drive position and velocity.
    // for future reference, only one of these lines is technically needed because
    // REVLib works
    // with status frames internally and these two settings correspond to the same
    // one.
    driveConfig.signals.primaryEncoderPositionPeriodMs((int) (1000 / DriveConstants.odometryFrequency))
        .primaryEncoderVelocityPeriodMs((int) (1000 / DriveConstants.odometryFrequency));
    steerConfig = getDefaultMaxConfig();
    steerConfig.inverted(true).smartCurrentLimit(60).encoder.quadratureMeasurementPeriod(8)
        .quadratureAverageDepth(2);
    shooterLeaderConfig = getDefaultFlexConfig();
    shooterLeaderConfig.encoder.quadratureAverageDepth(4).quadratureMeasurementPeriod(16);
    shooterLeaderConfig.smartCurrentLimit(80, 80).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.0004, 0, 0.003, ClosedLoopSlot.kSlot0).pid(0.0006, 0, 0, ClosedLoopSlot.kSlot1)
        .pid(0.00021, 0, 0, ClosedLoopSlot.kSlot2).pid(0.0001, 0, 0, ClosedLoopSlot.kSlot3).feedForward
            .kV(0.002, ClosedLoopSlot.kSlot0).kA(0.0001, ClosedLoopSlot.kSlot0)
            .kV(0.002, ClosedLoopSlot.kSlot1).kA(0.002, ClosedLoopSlot.kSlot1)
            .kV(Switchable.of(0.00181).addAlt(RobotTypes.duex26, 0.00188).get(), ClosedLoopSlot.kSlot2)
            .kV(0.0019, ClosedLoopSlot.kSlot3);
    shooterLeaderConfig.closedLoop.maxMotion.maxAcceleration(4000, ClosedLoopSlot.kSlot0).maxAcceleration(4000,
        ClosedLoopSlot.kSlot1);
    shooterLeaderConfig.signals.primaryEncoderVelocityPeriodMs(10).appliedOutputPeriodMs(10);
    shooterFollowerConfig = (SparkFlexConfig) getDefaultFlexConfig().follow(ShooterConstants.canId, true);
    hoodConfig = getDefaultMaxConfig();
    hoodConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1.6, 0, 0, ClosedLoopSlot.kSlot0);
    hoodConfig.inverted(true).absoluteEncoder.inverted(true);
    spindexerConfig = (SparkMaxConfig) getDefaultMaxConfig().inverted(SpindexerConstants.indexerSparkInverted);
    spindexerConfig.openLoopRampRate(0.5).smartCurrentLimit(80, 80);
    intakeConfig = (SparkMaxConfig) new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(false);
    intakeConfig.openLoopRampRate(0.5).smartCurrentLimit(60, 25);
    intakeRollerConfig = getDefaultMaxConfig();
    intakeRollerConfig.inverted(true);
    kickerConfig = getDefaultFlexConfig();
    kickerConfig.openLoopRampRate(0.8).smartCurrentLimit(80, 80);
    kickerConfig.encoder.quadratureAverageDepth(4).quadratureMeasurementPeriod(16);
    kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.00021, 0, 0,
        ClosedLoopSlot.kSlot0).feedForward.kV(0.001862, ClosedLoopSlot.kSlot0);
    turretConfig = getDefaultMaxConfig();
    turretConfig.inverted(true);
    turretConfig.idleMode(IdleMode.kBrake);
    climberConfig = getDefaultFlexConfig();
    climberConfig.idleMode(IdleMode.kBrake).closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1, 0, 0,
        ClosedLoopSlot.kSlot0);
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
