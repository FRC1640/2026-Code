package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

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

  /*
   * Commands
   */

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> io.setIndexerMotorVoltage(voltage.getAsDouble())).finallyDo(this::stop);
  }

  private void stop() {
    io.setIndexerMotorVoltage(0);
  }

  @Override
  public Command dashboardCommand(DoubleSupplier leftJoystickValue, DoubleSupplier rightJoystickValue) {
    return runVoltageCommand(() -> leftJoystickValue.getAsDouble() * -8);
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
