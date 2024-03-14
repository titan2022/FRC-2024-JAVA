package frc.robot.commands.shooter;

import static frc.robot.utility.Constants.Unit.DEG;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterControlCommand extends Command {
    private static final int COAST_TIME = 2000 / 50; // In frames (20ms)
    private ShooterSubsystem shooter;
    private XboxController xbox;

    private double shooterAngle = ShooterSubsystem.ANGLE_OFFSET;
    private boolean isCoasting = false;
    private int coastTimer = 0;
    private int shooterDir = 1;

    public ShooterControlCommand(ShooterSubsystem shooter, XboxController xbox) {
        this.shooter = shooter;
        this.xbox = xbox;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("targetAngle", ShooterSubsystem.MIN_ANGLE);
    }

    @Override
    public void execute() {
        shooterAngle = SmartDashboard.getNumber("targetAngle", ShooterSubsystem.MIN_ANGLE);//+= xbox.getRightY() * 5 * DEG * 0.02;
        shooterAngle = Math.min(shooterAngle, ShooterSubsystem.MAX_ANGLE);
        shooterAngle = Math.max(shooterAngle, ShooterSubsystem.MIN_ANGLE);
        shooter.setRotation(shooterAngle);

        if (xbox.getXButton()) {
            shooterDir = -1;
        } else {
            shooterDir = 1;
        }

        shooter.shoot(xbox.getRightTriggerAxis() * 0.8 * shooterDir);

        if (xbox.getRightBumperPressed()) {
            shooter.index(0.5);
        } else if (xbox.getRightBumperReleased()) {
            isCoasting = true;
        } else if (!xbox.getRightBumper() && coastTimer > COAST_TIME) {
            coastTimer = 0;
            isCoasting = false;
            shooter.index(0);
        }

        if (isCoasting) {
            coastTimer++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.holdAngle();
        shooter.holdIndex();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
