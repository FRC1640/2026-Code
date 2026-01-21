package frc.robot.subsystems.frank;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private double setpoint = 0.7;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }
  
    public Command setIntakePositionCommand(double pos) {
        return run(() -> io.setMotorPosition(pos, inputs))
            .finallyDo(() -> io.setMotorVoltage(0, inputs));
    }

    public Command setIntakePositionCommand(DoubleSupplier pos) {
        return run(() -> io.setMotorPosition(pos.getAsDouble(), inputs))
            .finallyDo(() -> io.setMotorVoltage(0, inputs));
    }

    public Command runIncrementedSetpoint(double change) {
        return setIntakePositionCommand(() -> setpoint)
            .beforeStarting(() -> {
                double newSetpoint = setpoint + change;
                if (!(newSetpoint > 0.9 || newSetpoint < 0.6)) {
                    setpoint = newSetpoint;
                }
                System.out.println(setpoint);
            });
    }
    
    public Command groundCommand() {
        return setIntakePositionCommand(IntakeConstants.intakeGroundPosition);
    }

    public Command handoffCommand() {
        return setIntakePositionCommand(IntakeConstants.intakeHandoffPosition);
    }

    public Command troughCommand() {
        return setIntakePositionCommand(IntakeConstants.troughPosition);
    }

    public Command safeCommand() {
        return setIntakePositionCommand(IntakeConstants.safePosition);
    }

    public boolean isIntakeRaised() {
        return isAtPosition(IntakeConstants.intakeHandoffPosition);
    }

    public boolean isAtTroughPosition() {
        return isAtPosition(IntakeConstants.troughPosition);
    }

    public boolean isAtPosition(double pos) {
        return Math.abs(inputs.encoderPosition - pos) < 0.04;
    }

    public double getPosition() {
        return inputs.encoderPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    /*---------------------------------
    | TEST COMMAND. NOT TO BE MERGED. |
    ---------------------------------*/
    public Command testVoltageCommand(boolean up) {
        return run(() -> io.setMotorVoltage(up ? 1 : -1, inputs)).finallyDo(() -> io.setMotorVoltage(0, inputs));
    }
}