package frc.robot.subsystems.shooter.turret;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {

  private SparkMax turretMotor;
  private SparkAnalogSensor turretEncoder;
  private PIDController turretController;

  public TurretIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(TurretConstants.canId, true);
    turretMotor = SparkConfigurer.configSparkMax(config);
    turretEncoder = turretMotor.getAnalog();
    turretController = RobotPIDConstants.constructPID(RobotPIDConstants.toyTurret);
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAngle = turretEncoder.getPosition() * TurretConstants.potToRadians - Math.PI;
    inputs.turretAngularVelocity = turretEncoder.getVelocity() * TurretConstants.potToRadians;
    inputs.turretMotorCurrent = turretMotor.getOutputCurrent();
    inputs.turretMotorVoltage = turretMotor.getBusVoltage() * turretMotor.getAppliedOutput();
    inputs.turretMotorTemperature = turretMotor.getMotorTemperature();
  }
}
