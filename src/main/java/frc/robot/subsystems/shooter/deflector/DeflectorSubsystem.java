package frc.robot.subsystems.shooter.deflector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;

public class DeflectorSubsystem extends SubsystemBase {

  private DeflectorIO io;
  private DeflectorIOInputsAutoLogged inputs = new DeflectorIOInputsAutoLogged();

  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.deflectorSubsystem;

  public DeflectorSubsystem(DeflectorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deflector", inputs);
  }

  public static DeflectorIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) {
      return new DeflectorIO() {

      };
    }
    return switch (Robot.getMode()) {
      case REAL -> new DeflectorIOReal();
      case SIM -> new DeflectorIOSim();
      case REPLAY -> new DeflectorIO() {
      };
    };
  }
}
