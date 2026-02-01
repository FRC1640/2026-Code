package frc.robot.subsystems.shooter.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {

  private SparkMax turretMotor;
  private AbsoluteEncoder turretEncoder;
  private SparkClosedLoopController turretController;

  public TurretIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(TurretConstants.canId, false);
    turretMotor = SparkConfigurer.configSparkMax(config);
    turretEncoder = turretMotor.getAbsoluteEncoder();
    turretController = turretMotor.getClosedLoopController();
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    turretController.setSetpoint(angularVelocity, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAngle = turretEncoder.getPosition() * 2 * Math.PI; // assuming 0-1 with zero point straight ahead
    inputs.turretAngularVelocity = turretEncoder.getVelocity() * 2 * Math.PI;
    inputs.turretMotorCurrent = turretMotor.getOutputCurrent();
    inputs.turretMotorVoltage = turretMotor.getBusVoltage() * turretMotor.getAppliedOutput();
    inputs.turretMotorTemperature = turretMotor.getMotorTemperature();
  }
}
