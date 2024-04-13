package frc.robot.commands.auto;

import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.utility.Constants.Unit.QUAD_ENCODER_CPR;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class HoldShooterRevCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double INDEX_SPEED = 0.5;
    // public static final double REV_DURATION = 3;
    public static final double REV_SPEED = 0.8;
    public static final double HOLD_SPEED = 0.7;
    public ShooterSubsystem shooter;
    public double speed;
    private boolean reached = false;
    // public double endTime;
    
    public HoldShooterRevCommand(double speed, ShooterSubsystem shooter) {
        this.shooter = shooter;
        this.speed = speed;

        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // endTime = Timer.getFPGATimestamp() + REV_DURATION;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // shooter.shoot(0.2);
        if(!reached) shooter.shoot(REV_SPEED);
        else{
            shooter.shoot(0.5);
            shooter.setShooterCoast();
        }
        reached = shooter.getShooterVelocity() >= speed;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // shooter.shootCoastToggle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;        
    }
}