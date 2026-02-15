package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class HoodIOSim implements HoodIO {
  private DCMotorSim hoodMotor;
  private PIDController angleController;
  // private PIDController velocityController;

  public HoodIOSim() {
    DCMotor gearboxSim = DCMotor.getNEO(1);
    hoodMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearboxSim, 0.0002, 1), gearboxSim);
    angleController = RobotPIDConstants.constructPID(RobotPIDConstants.hoodAnglePidSim);
    // velocityController =
    // RobotPIDConstants.constructPID(RobotPIDConstants.hoodVelocityPidSim);
  }

  @Override
  public void setAngleRad(double angle) {
    double angleOutputVolts = angleController.calculate(hoodMotor.getAngularPositionRad(), angle);
    hoodMotor.setInputVoltage(VoltageLim.clampVoltage(angleOutputVolts));
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setInputVoltage(voltage);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodMotor.update(0.02);

    inputs.angleRadians = hoodMotor.getAngularPositionRad();
    inputs.motorCurrent = hoodMotor.getCurrentDrawAmps();
    inputs.motorVoltage = hoodMotor.getInputVoltage();
    inputs.motorTemperatureCelsius = 0;
  }
}
