package frc.robot.subsystems.climber;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ClimberIOSim implements ClimberIO {

  private final SparkFlexSim m_motor;
  private final SparkAbsoluteEncoderSim m_encoder;

  public ClimberIOSim() {
    DCMotor motorGearboxSim = DCMotor.getNeoVortex(1);
    SparkFlex flexSim = new SparkFlex(ClimberConstants.canId, MotorType.kBrushless);
    m_motor = new SparkFlexSim(flexSim, motorGearboxSim);
    m_encoder = m_motor.getAbsoluteEncoderSim();
  }
  @Override
  public void setPosition(double position) {
      m_motor.setPosition(position);
  }

  @Override
  public void setVoltage(double voltage) {
     m_motor.setVelocity(voltage);
  }
    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        m_motor.iterate(
                Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                    0),
                RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                0.02);
        inputs.encoderPosition = m_encoder.getPosition();
        inputs.encoderVelocity = m_encoder.getVelocity();
        inputs.motorCurrent = m_motor.getMotorCurrent();
        inputs.motorTemperature = 0;
        inputs.motorVoltage = m_motor.getVelocity();
    }
}
