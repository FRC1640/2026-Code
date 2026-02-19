package frc.robot.util.spark;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkConfigurer {
  public static final SparkMax configSparkMax(int id, SparkMaxConfig config) {
    SparkMax spark = new SparkMax(id, MotorType.kBrushless);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return spark;
  }

  public static final SparkFlex configSparkFlex(int id, SparkFlexConfig config) {
    SparkFlex spark = new SparkFlex(id, MotorType.kBrushless);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return spark;
  }

  public static SparkMax configSparkMax(SparkConfiguration config) {
    SparkMax spark = new SparkMax(config.getId(), MotorType.kBrushless);
    boolean flash = getFlash(config, spark);
    spark.configure(config.getInnerConfig(), com.revrobotics.ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + config.getId(), flash);
    return spark;
  }

  /*
   * Configure a SparkMax with given SparkConfiguration config and what Leader it
   * is set to
   */
  public static SparkMax configSparkMax(SparkConfiguration config, SparkMax leader) {
    config.follow(leader);
    SparkMax spark = configSparkMax(config);
    return spark;
  }

  /*
   * Configure a SparkMax with given SparkConfiguration config and what Leader it
   * is set to
   */
  public static SparkMax configSparkMax(SparkConfiguration config, SparkMax leader, boolean inverted) {
    config.follow(leader, inverted);
    return configSparkMax(config);
  }

  public static SparkMax configSparkMax(SparkConfiguration config, SparkFlex leader) {
    config.follow(leader);
    return configSparkMax(config);
  }

  public static SparkMax configSparkMax(SparkConfiguration config, SparkFlex leader, boolean inverted) {
    config.follow(leader, inverted);
    return configSparkMax(config);
  }

  /*
   * Configure a spark flex with the given configuration
   */
  public static SparkFlex configSparkFlex(SparkConfiguration config) {
    SparkFlex spark = new SparkFlex(config.getId(), MotorType.kBrushless);
    boolean flash = getFlash(config, spark);
    spark.configure(config.getInnerConfig(), ResetMode.kResetSafeParameters,
        flash ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
    Logger.recordOutput("SparkFlashes/" + config.getId(), flash);
    return spark;
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config, SparkMax leader) {
    config.follow(leader);
    return configSparkFlex(config);
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config, SparkMax leader, boolean inverted) {
    config.follow(leader, inverted);
    return configSparkFlex(config);
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config, SparkFlex leader) {
    config.follow(leader);
    return configSparkFlex(config);
  }

  public static SparkFlex configSparkFlex(SparkConfiguration config, SparkFlex leader, boolean inverted) {
    config.follow(leader, inverted);
    return configSparkFlex(config);
  }

  /*
   * Check if the spark needs to be flashed for settings that are currently
   * flashed
   */

  private static boolean getFlash(SparkConfiguration config, SparkMax spark) {
    boolean flash = ((config.isInverted() != spark.configAccessor.getInverted())
        || (config.getIdleMode() != spark.configAccessor.getIdleMode())
        || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
        || (config.getEncoderMeasurementPeriod() != spark.configAccessor.encoder
            .getQuadratureMeasurementPeriod())
        || (config.getEncoderAverageDepth() != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    // if (config.getPID().isPresent()) {
    // flash =
    // (flash
    // || (config.getPID().get().kP != spark.configAccessor.closedLoop.getP())
    // || (config.getPID().get().kI != spark.configAccessor.closedLoop.getI())
    // || (config.getPID().get().kD != spark.configAccessor.closedLoop.getD()));
    // }
    return flash;
  }
  /*
   * Check if the spark needs to be flashed for settings that are currently
   * flashed
   */

  private static boolean getFlash(SparkConfiguration config, SparkFlex spark) {
    boolean flash = ((config.isInverted() != spark.configAccessor.getInverted())
        || (config.getIdleMode() != spark.configAccessor.getIdleMode())
        || (config.getCurrentLimit() != spark.configAccessor.getSmartCurrentLimit())
        || (config.getEncoderMeasurementPeriod() != spark.configAccessor.encoder
            .getQuadratureMeasurementPeriod())
        || (config.getEncoderAverageDepth() != spark.configAccessor.encoder.getQuadratureAverageDepth()));
    // if (config.getPID().isPresent()) {
    // flash =
    // (flash
    // || (config.getPID().get().kP != spark.configAccessor.closedLoop.getP())
    // || (config.getPID().get().kI != spark.configAccessor.closedLoop.getI())
    // || (config.getPID().get().kD != spark.configAccessor.closedLoop.getD()));
    // }
    return flash;
  }
}
