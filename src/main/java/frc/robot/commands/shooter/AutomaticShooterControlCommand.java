package frc.robot.commands.shooter;

import static frc.robot.utility.Constants.Unit.DEG;
import static frc.robot.utility.Constants.Unit.SECONDS;
import static frc.robot.utility.Constants.Unit.METERS;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utility.Localizer;

public class AutomaticShooterControlCommand extends Command {
    private static final int COAST_TIME = 2000 / 50; // In frames (20ms)
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private Localizer localizer;
    private XboxController xbox;
    private DataLog log;

    private DoubleLogEntry angleLog;
    private BooleanLogEntry shotLog;

    private double shooterAngle = ShooterSubsystem.ANGLE_OFFSET;
    private double offset = 0;
    private boolean isReadingOffsetIncrements = false;
    private static final double SHOOTER_SPEED = 5 * METERS / SECONDS;
    // private int shooterDir = 1;

    public AutomaticShooterControlCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, Localizer localizer, XboxController xbox, DataLog log) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.localizer = localizer;
        this.xbox = xbox;
        this.log = log;
        this.offset = 0;
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
        // if (Math.abs(xbox.getRightY()) > 0.1) {
        //     shooterAngle += -xbox.getRightY() * 80 * DEG * 0.02;// = SmartDashboard.getNumber("targetAngle", ShooterSubsystem.MIN_ANGLE);
        //     shooterAngle = Math.min(shooterAngle, ShooterSubsystem.MAX_ANGLE);
        //     shooterAngle = Math.max(shooterAngle, ShooterSubsystem.MIN_ANGLE);
            
        //     // shooter.setRotation(55 * DEG);
        //     // shooter.holdAngle();
        // }

        // Offsets from d-pad
        int dpad = xbox.getPOV();
        if(isReadingOffsetIncrements) {
            // If not pressed
            if(dpad < 0) {
                isReadingOffsetIncrements = false;
            }
        } else {
            // Up
            if(dpad > 270 || (dpad < 90 && dpad > 0)) {
                offset += 1 * DEG;
            // Down
            } else if(dpad < 270 && dpad > 90) {
                offset -= 1 * DEG;
            }
        }
        // Reset
        if(xbox.getLeftBumper()) {
            offset = 0;
        }

        // double finalAngle = shooterAngle + offset;
        // // shooterAngle = 65;
        // shooter.setRotation(finalAngle);

        if (xbox.getLeftTriggerAxis() > 0.5) {
            new ShooterSpeakerAlgCommand(SHOOTER_SPEED, shooter, indexer, localizer).alongWith(new RevShooterCommand(SHOOTER_SPEED, shooter));
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
