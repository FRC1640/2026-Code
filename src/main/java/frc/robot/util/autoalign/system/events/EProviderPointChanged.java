package frc.robot.util.autoalign.system.events;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.eventbus.events.IEvent;

public class EProviderPointChanged implements IEvent {
    public final Pose2d current;
    public final Pose2d previous;

    public EProviderPointChanged(Pose2d previous, Pose2d current) {
        this.previous = previous;
        this.current = current;
    }
}
