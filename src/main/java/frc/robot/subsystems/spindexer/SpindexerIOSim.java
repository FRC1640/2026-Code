package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class SpindexerIOSim implements SpindexerIO {
  private final DCMotorSim m_motorSim;
  private final PIDController m_velocityController;

  public SpindexerIOSim() {
    DCMotor motorGearboxSim = DCMotor.getNEO(1);

    m_motorSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(motorGearboxSim, 0.0002, SpindexerConstants.indexerGearRatioSim),
        motorGearboxSim);
    m_velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.spindexerVelocityPidSim);
  }

  @Override
  public void setVelocityRPM(double velocityRPM) {
    Logger.recordOutput("Subsystems/Spindexer/setpointVelocityRPM", velocityRPM);
    Logger.recordOutput("Subsystems/Spindexer/setpointVelocityRadPerSec", velocityRPM * Math.PI / 30);
    setVoltage(m_velocityController.calculate(m_motorSim.getAngularVelocityRPM(), velocityRPM));
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Spindexer/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    Logger.recordOutput("Subsystems/Spindexer/setpointVoltage", voltageClamped);
    m_motorSim.setInputVoltage(voltageClamped);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    m_motorSim.update(0.02);

    inputs.motorVelocityRadPerSec = m_motorSim.getAngularVelocityRadPerSec();
    inputs.motorVelocityRPM = m_motorSim.getAngularVelocityRPM();
    inputs.motorVoltage = m_motorSim.getInputVoltage();
    inputs.motorCurrent = m_motorSim.getCurrentDrawAmps();
    inputs.motorTemperatureCelsius = 0.0;
    inputs.isJammed = false;
    inputs.motorTotalEnergy += inputs.motorCurrent * inputs.motorVoltage * 0.02;
    inputs.motorPower = inputs.motorCurrent * inputs.motorVoltage;

  }
}
