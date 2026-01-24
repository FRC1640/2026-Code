package frc.robot.subsystems.indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class IndexerIOReal implements IndexerIO {
  private final SparkMax indexerSpark;
  private final RelativeEncoder indexerEncoder;
  public IndexerIOReal() {
    indexerSpark = SparkConfigurer.configSparkMax(SparkConstants.getDefaultMax(IndexerConstants.indexerSparkCanId,
        IndexerConstants.indexerSparkInverted));
    indexerEncoder = indexerSpark.getEncoder();
  }

  @Override
  public void setIndexerMotorVoltage(double voltage) {
    indexerSpark.setVoltage(voltage);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerMotorVelocity = indexerEncoder.getVelocity();
    inputs.indexerMotorVoltage = indexerSpark.getAppliedOutput();
    inputs.indexerMotorCurrent = indexerSpark.getOutputCurrent();
    inputs.indexerMotorTemperature = indexerSpark.getMotorTemperature();
  }

  @Override
  public void resetEncoder() {
    indexerEncoder.setPosition(0);
  }
}
