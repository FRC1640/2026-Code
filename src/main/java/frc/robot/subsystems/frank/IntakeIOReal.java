package frc.robot.subsystems.frank;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.util.limits.VoltageLim;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.SparkConfigurer;
import frc.robot.util.spark.SparkConstants;
import frc.robot.util.spark.SparkPIDConstants;

public class IntakeIOReal implements IntakeIO {
    private SparkMax intakeMotor;
    private AbsoluteEncoder encoder;
    private SparkClosedLoopController pid;

    public IntakeIOReal() {
        SparkConfiguration config = SparkConstants.getDefaultMax(IntakeConstants.canID, true)
          .applyPIDConfig(new SparkPIDConstants(0.01, 0, 0, 60, 0, ClosedLoopSlot.kSlot0));
        config.getInnerConfig().closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(3, 0, 0, ClosedLoopSlot.kSlot0).maxMotion
                .cruiseVelocity(200, ClosedLoopSlot.kSlot0)
                .maxAcceleration(260, ClosedLoopSlot.kSlot0)
                .allowedProfileError(0.01, ClosedLoopSlot.kSlot0);
        config.getInnerConfig().closedLoop.feedForward.kV(0);
        intakeMotor = SparkConfigurer.configSparkMax(config);
        encoder = intakeMotor.getAbsoluteEncoder();
        pid = intakeMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorTemperature = intakeMotor.getMotorTemperature();
        inputs.motorCurrent = intakeMotor.getOutputCurrent();
        inputs.motorVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.encoderPosition = encoder.getPosition();
        inputs.encoderVelocity = encoder.getVelocity();
        Logger.recordOutput("IntakeRelativePos", intakeMotor.getEncoder().getPosition());
    }

    @Override
    public void setMotorVoltage(double voltage, IntakeIOInputs inputs) {
        intakeMotor.setVoltage(VoltageLim.clampVoltage(voltage));
    }

    @Override
    public void setMotorPosition(double pos, IntakeIOInputs inputs) {
        // setMotorVoltage(VoltageLim.applyLimits(
        //     inputs.encoderPosition,
        //     intakePID.calculate(inputs.encoderPosition, pos),
        //     IntakeConstants.intakeLowerLimit,
        //     IntakeConstants.intakeUpperLimit), inputs);
        if (pos > IntakeConstants.intakeUpperLimit || pos < IntakeConstants.intakeLowerLimit) {
            intakeMotor.setVoltage(0);
        }
        pid.setSetpoint(pos, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }
}