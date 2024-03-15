package frc.robot.commands.shooter;

import static frc.robot.utility.Constants.Unit.DEG;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
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
    private DataLog log;

    private DoubleLogEntry angleLog;
    private BooleanLogEntry shotLog;

    private double shooterAngle = ShooterSubsystem.ANGLE_OFFSET;
    // private int shooterDir = 1;

    public ShooterControlCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, XboxController xbox, DataLog log) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.xbox = xbox;
        this.log = log;
        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("targetAngle", ShooterSubsystem.MIN_ANGLE);
        shooter.setRotation(55 * DEG);

        angleLog = new DoubleLogEntry(log, "/my/angle");
        shotLog = new BooleanLogEntry(log, "/my/shot");
    }

    @Override
    public void execute() {
        if (Math.abs(xbox.getRightY()) > 0.1) {
            shooterAngle += -xbox.getRightY() * 80 * DEG * 0.02;// = SmartDashboard.getNumber("targetAngle", ShooterSubsystem.MIN_ANGLE);
            shooterAngle = Math.min(shooterAngle, ShooterSubsystem.MAX_ANGLE);
            shooterAngle = Math.max(shooterAngle, ShooterSubsystem.MIN_ANGLE);
            
            // shooter.setRotation(55 * DEG);
            // shooter.holdAngle();
        }
        // shooterAngle = 65;
        shooter.setRotation(shooterAngle);
        

        if (xbox.getRightTriggerAxis() > 0.5) {
            double shooterMag = 0.8;//xbox.getRightTriggerAxis() * 0.5 * shooterDir;
            shooter.shoot(shooterMag);
            shotLog.append(true);
            angleLog.append(shooterAngle);
        } else {
            shooter.shoot(0);
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
