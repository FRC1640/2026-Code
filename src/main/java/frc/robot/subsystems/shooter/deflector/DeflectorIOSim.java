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
    deflectorMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxSim, 0.0002, 1), gearboxSim);
    angleController = RobotPIDConstants.constructPID(RobotPIDConstants.deflectorAnglePidSim);
    // velocityController =
    // RobotPIDConstants.constructPID(RobotPIDConstants.deflectorVelocityPidSim);
  }

  @Override
  public void setAngle(double angle) {
    double angleOutputVolts = angleController.calculate(deflectorMotor.getAngularPositionRad(), angle);
    deflectorMotor.setInputVoltage(VoltageLim.clampVoltage(angleOutputVolts));
  }

  @Override
  public void setDeflectorMotorVoltage(double voltage){
    deflectorMotor.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(DeflectorIOInputs inputs) {
    deflectorMotor.update(0.02);

    inputs.angle = deflectorMotor.getAngularPositionRad();
    inputs.motorCurrent = deflectorMotor.getCurrentDrawAmps();
    inputs.motorVoltage = deflectorMotor.getInputVoltage();
    inputs.motorTemperature = 0;
  }
}
