package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class RevShooterCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double INDEX_SPEED = 0.5;
    // public static final double REV_DURATION = 3;
    public static final double SHOOT_SPEED = 0.8;
    private ShooterSubsystem shooter;
    private LEDSubsystem led;
    public double speed;
    // public double endTime;
    
    public RevShooterCommand(double speed, ShooterSubsystem shooter, LEDSubsystem led) {
        this.shooter = shooter;
        this.speed = speed;
        this.led = led;

        // addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // endTime = Timer.getFPGATimestamp() + REV_DURATION;
        led.fill(255, 255, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.shoot(SHOOT_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.shoot(0);
        // shooter.shootCoastToggle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shooter.getShooterVelocity() >= speed;
        
    }
}