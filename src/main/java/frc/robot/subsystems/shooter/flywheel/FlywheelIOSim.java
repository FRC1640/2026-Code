package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotorSim m_motor;
  private final PIDController m_velocityController;

  public FlywheelIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxSim, 0.0002, 1), gearboxSim);
    m_velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.flywheelVelocityPidSim);
  }

  @Override
  public void setVelocity(double speedRadPerSec) {
    double outputVolts = m_velocityController.calculate(m_motor.getAngularVelocityRadPerSec(), speedRadPerSec);
    m_motor.setInputVoltage(VoltageLim.clampVoltage(outputVolts));
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setInputVoltage(VoltageLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    m_motor.update(0.02);

    inputs.leaderVelocity = m_motor.getAngularVelocityRadPerSec();
    inputs.followerVelocity = m_motor.getAngularVelocityRadPerSec();
    inputs.leaderMotorCurrent = m_motor.getCurrentDrawAmps();
    inputs.followerMotorCurrent = m_motor.getCurrentDrawAmps();
    inputs.leaderMotorVoltage = m_motor.getInputVoltage();
    inputs.followerMotorVoltage = m_motor.getInputVoltage();
    inputs.leaderMotorTemperature = 0;
    inputs.followerMotorTemperature = 0;
    inputs.averageVoltage = m_motor.getInputVoltage();
  }
}
