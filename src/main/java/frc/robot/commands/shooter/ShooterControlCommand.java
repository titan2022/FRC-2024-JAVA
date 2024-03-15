package frc.robot.commands.shooter;

import static frc.robot.utility.Constants.Unit.DEG;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterControlCommand extends Command {
    private static final int COAST_TIME = 2000 / 50; // In frames (20ms)
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private XboxController xbox;

    private double shooterAngle = ShooterSubsystem.ANGLE_OFFSET;
    private int shooterDir = 1;

    public ShooterControlCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, XboxController xbox) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.xbox = xbox;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("targetAngle", ShooterSubsystem.MIN_ANGLE);
        shooter.setRotation(55 * DEG);
    }

    @Override
    public void execute() {
        if (xbox.getRightStickButton()) {
            shooter.setRotation(65);
        } else {
            shooterAngle += -xbox.getRightY() * 40 * DEG * 0.02;// = SmartDashboard.getNumber("targetAngle", ShooterSubsystem.MIN_ANGLE);
            shooterAngle = Math.min(shooterAngle, ShooterSubsystem.MAX_ANGLE);
            shooterAngle = Math.max(shooterAngle, ShooterSubsystem.MIN_ANGLE);
            shooter.setRotation(shooterAngle);
            
            // shooter.setRotation(55 * DEG);
            // shooter.holdAngle();
        }

        if (xbox.getXButton()) {
            shooterDir = -1;
        } else {
            shooterDir = 1;
        }

        shooter.shoot(xbox.getRightTriggerAxis() * 0.5 * shooterDir);
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
