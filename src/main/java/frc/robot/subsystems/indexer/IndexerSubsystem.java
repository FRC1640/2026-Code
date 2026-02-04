package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.Subsystems;
import frc.robot.util.wrapper.subsystem.SubsystemInfo;
import frc.robot.util.wrapper.subsystem.SubsystemPlatform;

public class IndexerSubsystem extends SubsystemPlatform {

  IndexerIO io;
  IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  // THIS LINE IS ESSENTIAL FOR EVERY SUBSYSTEM
  public static final SubsystemInfo info = Subsystems.indexerSubsystem;

  public IndexerSubsystem(IndexerIO io) {
    super();
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public static IndexerIO getIOByMode() {
    if (!RobotConstants.RobotInformation.robot.isEnabled(info)) {
      return new IndexerIO() {

      };
    }
    return switch (Robot.getMode()) {
      case REAL -> new IndexerIOReal();
      case SIM -> new IndexerIOSim();
      case REPLAY -> new IndexerIO() {
      };
    };
  }
}
