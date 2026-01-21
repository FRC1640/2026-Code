package frc.robot.subsystems.shooter.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

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
  public void setTurretState(TurretSetpoint setpoint) {
    turretController.setSetpoint(setpoint.turretAngle(), ControlType.kMAXMotionPositionControl);
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
