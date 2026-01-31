package frc.robot.subsystems.shooter.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.MotorLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;

public class TurretIOReal implements TurretIO {
  private SparkMax turretMotor;
  private SparkAnalogSensor turretEncoder;
  private RelativeEncoder turretRelativeEncoder;
  private PIDController turretController;

  public TurretIOReal() {
    SparkConfiguration config = SparkConstants.getDefaultMax(TurretConstants.canId, true);
    turretMotor = SparkConfigurer.configSparkMax(config);
    turretEncoder = turretMotor.getAnalog();
    turretController = RobotPIDConstants.constructPID(RobotPIDConstants.toyTurret);
    turretRelativeEncoder = turretMotor.getEncoder();
  }

  @Override
  public void setTurretState(double angle, double angularVelocity) {
    double clampedAngle = TurretConstants.turretAngleLimits.clampPosition(angle);
    setTurretVoltage(turretController.calculate(getTurretPosition(), clampedAngle));
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turretMotor.setVoltage(MotorLim.clampVoltage(voltage));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAngle = getTurretPosition();
    inputs.turretAngularVelocity = turretEncoder.getVelocity() * TurretConstants.potToRadians;
    inputs.turretMotorCurrent = turretMotor.getOutputCurrent();
    inputs.turretMotorVoltage = turretMotor.getBusVoltage() * turretMotor.getAppliedOutput();
    inputs.turretMotorTemperature = turretMotor.getMotorTemperature();
  }

  private double getTurretPosition() {
    return turretEncoder.getPosition() * TurretConstants.potToRadians - Math.PI;
  }

  public boolean isSensorDisconnected() {
    return Math.abs(turretRelativeEncoder.getVelocity()) > 10 && Math.abs(turretEncoder.getVelocity())<0.01;
  }
}
