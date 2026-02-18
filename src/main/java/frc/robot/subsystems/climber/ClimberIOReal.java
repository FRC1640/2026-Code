package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;

public class ClimberIOReal implements ClimberIO {
  private final SparkFlex m_motor;
  private final AbsoluteEncoder m_encoder;

  public ClimberIOReal() {
    
  }
}
