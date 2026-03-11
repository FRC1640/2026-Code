package frc.robot.util.rumbler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.periodic.PeriodicBase;

public class Rumbler extends PeriodicBase{
    CommandXboxController controller;

    public Rumbler(CommandXboxController controller){
        this.controller = controller;
    }

    @Override
    public void periodic() {
        double time = DriverStation.getMatchTime();
        double[] times = {130, 105, 80, 55, 30};
        double[] offsets = {4,2,0};
        boolean toRumble = false;
        outerloop:
        for (double checkTime:times){
            for(double off:offsets){
                if (time == checkTime+off){
                    toRumble = true;
                    break outerloop;
                }
            }
        }
        if (toRumble) {
            controller.setRumble(RumbleType.kBothRumble, 1);
        } else {
            controller.setRumble(RumbleType.kBothRumble, 1);
        }
    }
    
}
