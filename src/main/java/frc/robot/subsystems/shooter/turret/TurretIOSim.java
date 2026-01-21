package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.subsystems.shooter.ShooterControl.TurretSetpoint;
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
  public void setTurretState(TurretSetpoint setpoint) {
    double thetaOutputVolts = angleController.calculate(turretMotor.getAngularPositionRad(), setpoint.turretAngle());
    double omegaOutputVolts = velocityController.calculate(turretMotor.getAngularVelocityRadPerSec(), setpoint.turretOmega());
    double outputVolts = VoltageLim.clampVoltage(thetaOutputVolts + omegaOutputVolts);
    turretMotor.setInputVoltage(outputVolts);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAngle = turretMotor.getAngularPositionRad();
    inputs.turretAngularVelocity = turretMotor.getAngularVelocityRadPerSec();
    inputs.turretMotorCurrent = turretMotor.getCurrentDrawAmps();
    inputs.turretMotorVoltage = turretMotor.getInputVoltage();
    inputs.turretMotorTemperature = 0;
  }
}
