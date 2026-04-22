package frc.robot.util.controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.periodic.PeriodicBase;

public class RumbleManager extends PeriodicBase {
  private CommandXboxController controller;

  private Rumble lastRumble = null;

  private Rumble defaultRumble = new Rumble(0, RumbleType.kBothRumble);
  private Rumble rumbleOverride = null;

  public RumbleManager(CommandXboxController controller) {
    this.controller = controller;
  }

  public Command setRumbleCommand(double magnitude, RumbleType side) {
    return new InstantCommand(() -> setRumble(magnitude, side));
  }

  public Command rumbleCommand(double magnitude, RumbleType side) {
    return rumbleCommand(magnitude, side, false);
  }

  public Command rumbleCommand(double magnitude, RumbleType side, boolean restoreCurrentRumble) {
    return new WaitUntilCommand(() -> false).beforeStarting(() -> setRumble(magnitude, side, restoreCurrentRumble))
        .finallyDo(() -> loadLastRumble());
  }

  public Command rumblePulseCommand(double magnitude, RumbleType side, double duration) {
    return new WaitCommand(duration).beforeStarting(() -> setRumbleOverride(magnitude, side))
        .finallyDo(() -> clearRumbleOverride());
  }

  private void setRumble(double magnitude, RumbleType side) {
    setRumble(magnitude, side, false);
  }

  private void setRumble(double magnitude, RumbleType side, boolean saveCurrentRumble) {
    if (saveCurrentRumble)
      lastRumble = defaultRumble.clone();
    else
      lastRumble = null;
    defaultRumble.magnitude = magnitude;
    defaultRumble.side = side;
  }

  private void setRumbleOverride(double magnitude, RumbleType side) {
    rumbleOverride = new Rumble(magnitude, side);
  }

  private void clearRumbleOverride() {
    rumbleOverride = null;
  }

  private void loadLastRumble() {
    if (lastRumble != null)
      defaultRumble = lastRumble;
    lastRumble = null;
  }

  @Override
  public void periodic() {
    if (rumbleOverride != null)
      controller.setRumble(rumbleOverride.side, rumbleOverride.magnitude);
    else
      controller.setRumble(defaultRumble.side, defaultRumble.magnitude);
  }

  private static class Rumble {
    public double magnitude;
    public RumbleType side;

    private Rumble(double magnitude, RumbleType side) {
      this.magnitude = magnitude;
      this.side = side;
    }

    public Rumble clone() {
      return new Rumble(magnitude, side);
    }
  }
}
