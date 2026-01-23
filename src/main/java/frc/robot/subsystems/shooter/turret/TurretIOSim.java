package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class TurretIOSim implements TurretIO {
  private DCMotorSim turretMotor;
  private PIDController angleController;
  private PIDController velocityController;

  public TurretIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    turretMotor = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        gearboxSim, 0.0002, 1),
      gearboxSim);
    angleController = RobotPIDConstants.constructPID(RobotPIDConstants.turretAnglePidSim);
    velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.turretVelocityPidSim);
  }
  
  @Override
  public void setTurretState(double angle, double angularVelocity) {
    double thetaOutputVolts = angleController.calculate(turretMotor.getAngularPositionRad(), angle);
    double omegaOutputVolts = velocityController.calculate(turretMotor.getAngularVelocityRadPerSec(), angularVelocity);
    double outputVolts = VoltageLim.clampVoltage(thetaOutputVolts + omegaOutputVolts);
    turretMotor.setInputVoltage(outputVolts);
  }

  @Override
  public void setTurretVoltage(double voltage) {
    turretMotor.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    turretMotor.update(0.02);

    inputs.turretAngle = turretMotor.getAngularPositionRad();
    inputs.turretAngularVelocity = turretMotor.getAngularVelocityRadPerSec();
    inputs.turretMotorCurrent = turretMotor.getCurrentDrawAmps();
    inputs.turretMotorVoltage = turretMotor.getInputVoltage();
    inputs.turretMotorTemperature = 0;
  }
}
