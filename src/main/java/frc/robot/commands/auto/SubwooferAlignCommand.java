package frc.robot.commands.auto;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.utility.Constants.Unit.DEG;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SubwooferAlignCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    public ShooterSubsystem shooter;
    public double DURATION = 2.7;
    public double endTime = 0;
    // public double endTime;
    
    public SubwooferAlignCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }
    @Override
    public void initialize() {
        shooter.setTargetAngle(new Rotation2d(60*DEG));
        // delayTime = Timer.getFPGATimestamp() + DELAY_TIME;
        endTime = Timer.getFPGATimestamp() + DURATION;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // shooter.shoot(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > endTime;
    }
}