package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

abstract public class AutoPlan {
    protected ArrayList<Command> commandList;
    protected boolean onBlueSide;

    abstract protected void defineCommands();

    public AutoPlan(boolean onBlueSide) {
        this.onBlueSide = onBlueSide;
        defineCommands();
        setSide();
    }

    protected void setSide() {
        for (int i = 0; i < commandList.size(); i++) {
            if (VariantCommand.class.isInstance(commandList.get(i))) {
                if (((VariantCommand) commandList).onBlueSide() != onBlueSide) {
                    ((VariantCommand) commandList).changeColorSide();
                }
            }
        }
    }

    public void run() {
        for (int i = 0; i < commandList.size(); ++i) {
            CommandScheduler.getInstance().schedule(commandList.get(i));
        }
    }
}
