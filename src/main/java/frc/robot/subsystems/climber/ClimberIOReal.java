package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class ClimberIOReal implements ClimberIO {
  private final SparkFlex m_motor;
  private final AbsoluteEncoder m_encoder;
  private final SparkClosedLoopController m_positionController;

  public ClimberIOReal() {
    m_motor = SparkConfigurer.configSparkFlex(ClimberConstants.canId, SparkConstants.climberConfig);
    m_encoder = m_motor.getAbsoluteEncoder();
    m_positionController = m_motor.getClosedLoopController();
  }

  @Override
  public void setPosition(double position) {
    m_positionController.setSetpoint(position, null, null);
  }

  @Override
  public void setVoltage(double voltage) {
    double voltageClamped = MotorLim.clampVoltage(voltage);
    voltageClamped = ClimberConstants.limits.clampOutput(m_encoder.getPosition(), voltageClamped);
    m_motor.setVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    
  }
}
