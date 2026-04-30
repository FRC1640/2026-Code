package frc.robot.util.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotCommands;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class AutonBuilder {

    private static AutonBuilder instance;

    public record Auton(Command command, RobotCommands robotCommands) {
        public Auton(Command command, RobotCommands robotCommands) {
            this.robotCommands = robotCommands;
            this.command = // Commands.sequence(
                    // robotCommands.setSteerPositionCommand(
                    // firstPath != null ? firstPath.getInitialModuleDirection() :
                    // Rotation2d.fromDegrees(180)),
                    (command).finallyDo(AutonBuilder.getInstance().autoEndCallback);

        }
    }

    public final HashMap<String, Auton> autons = new HashMap<String, Auton>();

    public final Runnable autoEndCallback;

    public AutonBuilder(RobotCommands robotCommands, FollowPath.Builder pathBuilder, Runnable autoEndCallback) {
        AutonBuilder.instance = this;
        this.autoEndCallback = autoEndCallback;

        SmartDashboard.putNumber("AutoWaitTime", 0.0);

        // custom format

        /*--------
        | AUTONS |
        --------*/

        // None
        autons.put("None", new Auton(Commands.none(), robotCommands));

        // autons.put("Center Outpost + Depot", new
        // Auton(Commands.sequence(pathBuilder.build(new Path("collect_outpost")),
        // new WaitCommand(2),
        // new InstantCommand(
        // () ->
        // CommandScheduler.getInstance().schedule(robotCommands.autoOscillateCommand(65,
        // 0))),
        // pathBuilder.build(new Path("outpost_depot"))), robotCommands));

        // autons.put("Outpost 2Sweep",
        // new Auton(
        // Commands.sequence(pathBuilder.build(new Path("outpost_2sweep")).finallyDo(
        // () ->
        // CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
        // robotCommands));

        // autons.put("Depot 2Sweep",
        // new Auton(
        // Commands.sequence(pathBuilder.build(new Path("depot_2sweep")).finallyDo(
        // () ->
        // CommandScheduler.getInstance().schedule(robotCommands.setSwerveToZeroCommand()))),
        // robotCommands));

        autons.put("Depot Double Sweep (FMA)",
                new Auton(
                        Commands.sequence(
                                robotCommands.autoDelayCommand(),
                                pathBuilder.build(new Path("depot_fma_2sweep")).finallyDo(
                                        () -> CommandScheduler.getInstance()
                                                .schedule(robotCommands.setSwerveToZeroCommand()))),
                        robotCommands));

        Path outpostfma2sPath = new Path("depot_fma_2sweep");
        outpostfma2sPath.mirror();

        autons.put("Outpost Double Sweep (FMA)",
                new Auton(
                        Commands.sequence(
                                robotCommands.autoDelayCommand(),
                                pathBuilder.build(outpostfma2sPath).finallyDo(
                                        () -> CommandScheduler.getInstance()
                                                .schedule(robotCommands.setSwerveToZeroCommand()))),
                        robotCommands));

        autons.put("Depot Trench Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(new Path("hub_bump_route")),
                        pathBuilder.build(new Path("hub_intake_return")),
                        pathBuilder.build(new Path("collect_depot"))),
                robotCommands));

        Path outposthbrPath = new Path("hub_bump_route");
        outposthbrPath.mirror();
        Path outposthirPath = new Path("hub_intake_return");
        outposthirPath.mirror();

        Path outpostt3bPath = new Path("depot_trench_3rd_bot");
        outpostt3bPath.mirror();

        autons.put("Outpost Trench Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(outposthbrPath),
                        pathBuilder.build(outposthirPath),
                        pathBuilder.build(new Path("collect_outpost"))),
                robotCommands));

        autons.put("Depot Bump Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(new Path("hub_trench_route")),
                        pathBuilder.build(new Path("hub_intake_return")),
                        pathBuilder.build(new Path("collect_depot"))),
                robotCommands));

        Path outposthtrPath = new Path("hub_trench_route");
        outposthtrPath.mirror();

        Path outpostb3bPath = new Path("depot_bump_3rd_bot");

        autons.put("Outpost Bump Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(outposthtrPath),
                        pathBuilder.build(outposthirPath),
                        pathBuilder.build(new Path("collect_outpost"))),
                robotCommands));

        autons.put("Spliced Depot Trench Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(new Path("depot_trench_3rd_bot")),
                        pathBuilder.build(new Path("collect_depot"))),
                robotCommands));
        autons.put("Spliced Outpost Trench Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(outpostt3bPath),
                        pathBuilder.build(new Path("collect_outpost"))),
                robotCommands));
        autons.put("Spliced Depot Bump Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(new Path("depot_bump_3rd_bot")),
                        pathBuilder.build(new Path("collect_depot"))),
                robotCommands));
        autons.put("Spliced Outpost Bump Third Robot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(outpostb3bPath),
                        pathBuilder.build(new Path("collect_outpost"))),
                robotCommands));

        // start at trench -> move out -> wait -> sweep and return through trench ->
        // depot
        /*
         * autons.put("Depot Bump Pair", new Auton(
         * Commands.sequence(
         * robotCommands.autoDelayCommand(),
         * pathBuilder.build(new Path("hub_bump_route")),
         * pathBuilder.build(new Path("depot_hub_trench_sweep")),
         * pathBuilder.build(new Path("trench_to_depot")),
         * pathBuilder.build(new Path("collect_depot"))),
         * robotCommands));
         */

        Path outpostt2oPath = new Path("trench_to_depot");
        outpostt2oPath.mirror();

        // start at trench -> move out -> wait -> sweep and return through trench ->
        // outpost
        /*
         * autons.put("Outpost Bump Pair", new Auton(
         * Commands.sequence(
         * robotCommands.autoDelayCommand(),
         * pathBuilder.build(outposthbrPath),
         * pathBuilder.build(new Path("outpost_hub_trench_sweep")),
         * pathBuilder.build(outpostt2oPath),
         * pathBuilder.build(new Path("collect_outpost"))),
         * robotCommands));
         */

        autons.put("Straight To Depot", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(new Path("bump_to_depot")),
                        pathBuilder.build(new Path("collect_depot"))),
                robotCommands));

        Path outpostb2oPath = new Path("bump_to_depot");
        outpostb2oPath.mirror();

        autons.put("Straight To Outpost", new Auton(
                Commands.sequence(
                        robotCommands.autoDelayCommand(),
                        pathBuilder.build(outpostb2oPath),
                        pathBuilder.build(new Path("collect_outpost"))),
                robotCommands));

        // TODO: test the following: Center 1Sweep Depot, Center 2Sweep Depot, Center
        // 1Sweep Outpost, Center 2Sweep Outpost, Straight To Outpost, Straight To
        // Depot, Depot 2Sweep, Outpost 2Sweep

        // add autons here!!!! MAKE SURE YOU PRESERVE THE HOOD AND PROPER
        // SHOOTERIDLE USE.

        // spotless format

    }

    public static AutonBuilder getInstance() {
        return instance;
    }
}
