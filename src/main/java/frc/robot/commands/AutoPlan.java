package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

abstract public class AutoPlan extends SequentialCommandGroup {
    protected ArrayList<Command> commandList;
    protected Alliance alliance;

    public AutoPlan(Alliance alliance) {
        this.alliance = alliance;
    }

    public void flipColor() {
        for (int i = 0; i < commandList.size(); i++) {
            if (VariantCommand.class.isInstance(commandList.get(i))) {
                ((VariantCommand) commandList.get(i)).changeColorSide();
            }
        }
    }

    public void run() {
        for (int i = 0; i < commandList.size(); ++i) {
            CommandScheduler.getInstance().schedule(commandList.get(i));
        }
    }
}
