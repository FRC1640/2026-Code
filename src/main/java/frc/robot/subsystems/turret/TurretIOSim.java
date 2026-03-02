package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim m_motor;
  private final PIDController m_angleController;
  private final PIDController m_velocityController;

  public TurretIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxSim, 0.0002, 1), gearboxSim);
    m_angleController = RobotPIDConstants.constructPID(RobotPIDConstants.turretAnglePidSim);
    m_velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.turretVelocityPidSim);
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    double thetaOutputVolts = m_angleController.calculate(m_motor.getAngularPositionRad(), angle);
    double omegaOutputVolts = m_velocityController.calculate(m_motor.getAngularVelocityRadPerSec(),
        angularVelocity);
    double outputVolts = VoltageLim.clampVoltage(thetaOutputVolts + omegaOutputVolts);
    m_motor.setInputVoltage(outputVolts);
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
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
