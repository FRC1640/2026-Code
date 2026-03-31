package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class AutonBuilder {

  private static AutonBuilder instance;

  public record Auton(Command command, Path firstPath) {
    public Auton(Command command, Path firstPath) {
      this.command = command;
      this.firstPath = firstPath;
    }
  }

  public final HashMap<String, Auton> autons = new HashMap<String, Auton>();

  public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder) {
    AutonBuilder.instance = this;

    // custom format

    /*--------
    | AUTONS |
    --------*/

    // None
    autons.put("None", new Auton(Commands.none(), null));

    // Example: Preload -> Near Hub -> Shoot for 8 seconds -> Outpost
    autons.put("Example", new Auton(
      Commands.sequence(
          pathBuilder.build(new Path("e1")),
          Commands.deadline(
              new WaitCommand(8),
              robotCommands.autoShootCommand()),
          pathBuilder.build(new Path("e2"))),
      new Path("e1")
    ));

    // TODO: add autons here!!!!

    // spotless format

  }

  public static AutonBuilder getInstance() {
    return instance;
  }
}
