package frc.robot.subsystems.shooter.deflector;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class DeflectorIOSim implements DeflectorIO {
  private DCMotorSim deflectorMotor;
  private PIDController angleController;
  // private PIDController velocityController;

  public DeflectorIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    deflectorMotor = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
        gearboxSim, 0.0002, 1),
      gearboxSim);
    angleController = RobotPIDConstants.constructPID(RobotPIDConstants.deflectorAnglePidSim);
    // velocityController = RobotPIDConstants.constructPID(RobotPIDConstants.deflectorVelocityPidSim);
  }

  @Override
  public void setDeflectorAngle(double angle) {
    double angleOutputVolts = angleController.calculate(deflectorMotor.getAngularPositionRad(), angle);
    deflectorMotor.setInputVoltage(VoltageLim.clampVoltage(angleOutputVolts));
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    inputs.deflectorAngle = deflectorMotor.getAngularPositionRad();
    inputs.deflectorMotorCurrent = deflectorMotor.getCurrentDrawAmps();
    inputs.deflectorMotorVoltage = deflectorMotor.getInputVoltage();
    inputs.deflectorMotorTemperature = 0;
  }
}
