package frc.robot.subsystems.shooter.deflector;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DeflectorSubsystem extends SubsystemBase {
  private DeflectorIO io;
  private DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  SysIdRoutine sysIdRoutine;

  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deflector", inputs);
  }
}
