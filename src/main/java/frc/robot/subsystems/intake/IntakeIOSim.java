package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.RobotPIDConstants;
import frc.robot.util.limits.VoltageLim;

public class IntakeIOSim implements IntakeIO {

  private final DCMotorSim m_motor;
  private final PIDController m_positionController = RobotPIDConstants.constructPID(RobotPIDConstants.intakeSim);
  private final SimpleMotorFeedforward m_feedforwardController = RobotPIDConstants
      .constructFFSimpleMotor(RobotPIDConstants.intakeFFSim);

  Mechanism2d mech;
  // the mechanism root node
  MechanismRoot2d root;
  MechanismLigament2d intakeLigament;

  public IntakeIOSim() {
    DCMotor gearbox = DCMotor.getNEO(1);
    m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.00019125, 1), gearbox);
    m_motor.setAngle(IntakeConstants.stowedPositionRadians);

    mech = new Mechanism2d(3, 3);
    root = mech.getRoot("intake", 1.5, 0);
    intakeLigament = root.append(new MechanismLigament2d("intake", 1, 0, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData("IntakeMech", mech);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    m_motor.update(0.02);

    inputs.motorTemperatureCelsius = 0; // degrees celsius
    inputs.motorCurrent = m_motor.getCurrentDrawAmps(); // amps
    inputs.motorVoltage = m_motor.getInputVoltage(); // volts
    inputs.positionRadians = m_motor.getAngularPositionRad(); // radians
    inputs.velocityRadPerSec = m_motor.getAngularVelocityRadPerSec(); // rad/s
    inputs.positionDegrees = inputs.positionRadians * 180 / Math.PI;
    inputs.velocityDegreesPerSec = inputs.velocityRadPerSec * 180 / Math.PI;
    inputs.motorDrawJoules += inputs.motorCurrent * inputs.motorVoltage * 0.02;
    inputs.motorWattage = inputs.motorCurrent * inputs.motorCurrent; // W

    intakeLigament.setAngle(90 - Units.radiansToDegrees(m_motor.getAngularPositionRad()));
  }

  @Override
  public void setVoltage(double voltage) {
    Logger.recordOutput("Subsystems/Intake/desiredVoltage", voltage);
    double voltageClamped = VoltageLim.clampVoltage(voltage);
    voltageClamped = IntakeConstants.positionLimitsRadians.clampOutput(m_motor.getAngularPositionRad(),
        voltageClamped);
    Logger.recordOutput("Subsystems/Intake/setpointVoltage", voltageClamped);
    m_motor.setInputVoltage(voltageClamped);
  }

  @Override
  public void setState(double angleRadians, double angularVelocityRadPerSec) {
    Logger.recordOutput("Subsystems/Intake/setpointRadians", angleRadians);
    Logger.recordOutput("Subsystems/Intake/setpointDegrees", angleRadians * 180 / Math.PI);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityRadPerSec", angularVelocityRadPerSec);
    Logger.recordOutput("Subsystems/Intake/setpointVelocityDegreesPerSec",
        angularVelocityRadPerSec * 180 / Math.PI);
    double voltage = m_positionController.calculate(m_motor.getAngularPositionRad(), angleRadians)
        + m_feedforwardController.calculate(angularVelocityRadPerSec);
    setVoltage(voltage);
  }
}
