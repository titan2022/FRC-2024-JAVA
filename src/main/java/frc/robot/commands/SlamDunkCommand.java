package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SlamDunkerSubsystem;

public class SlamDunkCommand  extends Command {
    private SlamDunkerSubsystem slamDunker;
    public SlamDunkCommand(SlamDunkerSubsystem slamDunker) {
        this.slamDunker = slamDunker;
    }
    
    int timer = 0;


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        timer+= 20;

        slamDunker.setIntake(10);

        if (timer == 2000){
            slamDunker.setIntake(0);
            slamDunker.setArm(10);
        }

        if (timer == 4000){
            slamDunker.setIntake(-10);
            slamDunker.setArm(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        slamDunker.setArm(0);
        slamDunker.setIntake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}