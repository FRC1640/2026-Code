package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim m_motor;
  private final PIDController m_positionController;

  public ClimberIOSim() {
    DCMotor motorGearboxSim = DCMotor.getNeoVortex(1);
    m_motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(motorGearboxSim, 0.0002, ClimberConstants.climberGearRatioSim),
        motorGearboxSim);
    m_positionController = RobotPIDConstants.constructPID(RobotPIDConstants.climberPidSim);
  }

  @Override
  public void setPosition(double position) {
    Logger.recordOutput("Subsystems/Climber/setpoint", position);
    double positionClamped = ClimberConstants.positionLimitsMeters.clampPosition(position);
    Logger.recordOutput("Subsystems/Climber/setpointClamped", positionClamped);
    setVoltage(m_positionController.calculate(m_motor.getAngularPositionRad(), positionClamped));
  }

  @Override
  public void setHeight(double height) {
    double position = (height - ClimberConstants.climberRetractedHeight)
        / Math.cos(ClimberConstants.climberAngleRadians);
    setPosition(position);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Climber/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = ClimberConstants.positionLimitsMeters.clampOutput(m_motor.getAngularPositionRad(),
        voltageClamped);
    Logger.recordOutput("Subsystems/Climber/clampedVoltage", voltageClamped);
    m_motor.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    m_motor.update(0.02);

    inputs.positionMeters = m_motor.getAngularPositionRad();
    inputs.velocityMetersPerSec = m_motor.getAngularVelocityRadPerSec();
    inputs.heightMeters = ClimberConstants.climberRetractedHeight
        + inputs.positionMeters * Math.cos(ClimberConstants.climberAngleRadians);
    inputs.verticalVelocityMetersPerSec = inputs.velocityMetersPerSec
        * Math.cos(ClimberConstants.climberAngleRadians);
    inputs.motorCurrent = m_motor.getCurrentDrawAmps();
    inputs.motorVoltage = m_motor.getInputVoltage();
    inputs.motorTemperature = 0;
  }
}
