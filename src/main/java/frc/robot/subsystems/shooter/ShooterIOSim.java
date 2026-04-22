package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim m_motor;
  private final PIDController m_velocityController;
  private final SimpleMotorFeedforward m_velocityFeedfoward;

  public ShooterIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxSim, 0.0002, 1), gearboxSim);
    m_velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.shooterVelocityPidSim);
    m_velocityFeedfoward = RobotPIDConstants.constructFFSimpleMotor(RobotPIDConstants.shooterVelocityFFSim);
  }

  @Override
  public void setVelocityRadPerSec(double velocityRadPerSec, double accelerationRadPerSecSquared) {
    Logger.recordOutput("Subsystems/Shooter/setpointVelocityRadPerSec", velocityRadPerSec);
    Logger.recordOutput("Subsystems/Shooter/setpointVelocityRPM", velocityRadPerSec * 60 / (2 * Math.PI));
    double outputVolts = m_velocityFeedfoward.calculate(velocityRadPerSec)
        + m_velocityController.calculate(m_motor.getAngularVelocityRadPerSec(), velocityRadPerSec);
    m_motor.setInputVoltage(outputVolts);
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Shooter/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    Logger.recordOutput("Subsystems/Shooter/setpointVoltage", voltageClamped);
    m_motor.setInputVoltage(voltageClamped);
  }

  @Override
  public boolean isAtSetpoint() {
    return true;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_motor.update(0.02);

    inputs.leaderVelocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
    inputs.leaderVelocityRPM = m_motor.getAngularVelocityRPM();
    inputs.leaderMotorCurrent = m_motor.getCurrentDrawAmps();
    inputs.leaderMotorVoltage = m_motor.getInputVoltage();
    inputs.leaderMotorTemperatureCelsius = 0;
    inputs.leaderMotorPositionRotations = m_motor.getAngularPositionRotations();

    inputs.followerVelocityRadPerSec = m_motor.getAngularVelocityRadPerSec();
    inputs.followerVelocityRPM = m_motor.getAngularVelocityRPM();
    inputs.followerMotorCurrent = m_motor.getCurrentDrawAmps();
    inputs.followerMotorVoltage = m_motor.getInputVoltage();
    inputs.followerMotorTemperatureCelsius = 0;
    inputs.followerMotorPositionRotations = m_motor.getAngularPositionRotations();
    inputs.leaderTotalEnergy += inputs.leaderMotorVoltage * inputs.leaderMotorCurrent * 0.02; // J
    inputs.followerTotalEnergy += inputs.followerMotorVoltage * inputs.followerMotorCurrent * 0.02; // J
    inputs.totalEnergy = inputs.leaderTotalEnergy + inputs.followerTotalEnergy; // J
    inputs.leaderPower = inputs.leaderMotorVoltage * inputs.leaderMotorCurrent; // W
    inputs.followerPower = inputs.followerMotorVoltage * inputs.followerMotorCurrent; // W
    inputs.totalPower = inputs.leaderPower + inputs.followerPower; // W
    inputs.averageVoltage = m_motor.getInputVoltage();
  }
}
