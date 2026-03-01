package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class HoodIOSim implements HoodIO {
  private final DCMotorSim m_motor;
  private final PIDController m_angleController;
  // private PIDController velocityController;

  public HoodIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxSim, 0.0002, 1), gearboxSim);
    m_angleController = RobotPIDConstants.constructPID(RobotPIDConstants.hoodAnglePidSim);
    // velocityController =
    // RobotPIDConstants.constructPID(RobotPIDConstants.hoodVelocityPidSim);
  }

  @Override
  public void setAngleRadians(double angle) {
    Logger.recordOutput("Subsystems/Hood/setpointRadians", angle);
    Logger.recordOutput("Subsystems/Hood/setpointDegrees", angle * 180 / Math.PI);
    double angleOutputVolts = m_angleController.calculate(m_motor.getAngularPositionRad(), angle);
    setVoltage(angleOutputVolts);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Hood/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = HoodConstants.angleLimitsRadians.clampOutput(m_motor.getAngularPositionRad(), voltageClamped);
    Logger.recordOutput("Subsystems/Hood/clampedVoltage", voltageClamped);
    m_motor.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    m_motor.update(0.02);

    inputs.angleRadians = m_motor.getAngularPositionRad();
    inputs.angularVelocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
    inputs.angleDegrees = inputs.angleRadians * 180 / Math.PI;
    inputs.angularVelocityDegreesPerSec = inputs.angularVelocityRadPerSec * 180 / Math.PI;
    inputs.motorCurrent = m_motor.getCurrentDrawAmps();
    inputs.motorVoltage = m_motor.getInputVoltage();
    inputs.motorTemperatureCelsius = 0;
  }
}
