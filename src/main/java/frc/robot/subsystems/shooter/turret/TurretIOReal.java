package frc.robot.subsystems.shooter.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;

public class TurretIOReal implements TurretIO {
  private SparkMax turretMotor;
  private AbsoluteEncoder turretEncoder;

  public TurretIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(TurretConstants.canId, false);
    turretMotor = SparkConfigurer.configSparkMax(config);
    turretEncoder = turretMotor.getAbsoluteEncoder();
  }

  @Override
  public void setTurretState(TurretSetpoint setpoint) {
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAngle = turretEncoder.getPosition() * 2 * Math.PI; // assuming 0-1 with zero point straight ahead
    inputs.turretAngularVelocity = turretEncoder.getVelocity() * 2 * Math.PI;
    inputs.turretMotorCurrent = turretMotor.getOutputCurrent();
    inputs.turretMotorTemperature = turretMotor.getMotorTemperature();
  }
}
