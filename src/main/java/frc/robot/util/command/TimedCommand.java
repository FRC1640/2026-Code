package frc.robot.util.command;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TimedCommand extends FunctionalCommand{

    public TimedCommand(Consumer<Timer> toRun, Subsystem... requirements){
        Timer t = new Timer();
        t.start();
        FunctionalCommand(() -> {}, () -> {toRun(t);}, interrupted -> {}, () -> false,  requirements);
        FunctionalCommand(() -> {}, () -> {}, interrupted -> {}, () -> false, requirements);
    }
    
}
