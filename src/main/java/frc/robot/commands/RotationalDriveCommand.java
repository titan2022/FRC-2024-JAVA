package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.RotationalDrivebase;
import frc.robot.utility.Localizer;

import static frc.robot.utility.Constants.Unit.*;

public class RotationalDriveCommand extends Command {
    private RotationalDrivebase drive;
    private XboxController xbox;
    private double maxRate, omega;
    private Localizer localizer;

    /**
     * Controls the rotational velocity of the drivebase with a joystick.
     * 
     * @param drivebase The drivebase to control.
     * @param localizer The localizer to use for orientation correction.
     * @param xbox      The joystick controller to use.
     * @param maxRate  The maximum rotational velocity in radians per second.
     */
    public RotationalDriveCommand(RotationalDrivebase drive, Localizer localizer, XboxController xbox, double maxRate) {
        this.drive = drive;
        this.localizer = localizer;
        this.xbox = xbox;
        this.maxRate = maxRate;
        addRequirements(drive);
    }

    /**
     * Controls the rotational velocity of the drivebase with a joystick.
     * 
     * @param drivebase The drivebase to control.
     * @param xbox      The joystick controller to use.
     * @param maxRate  The maximum rotational velocity in radians per second.
     */
    public RotationalDriveCommand(RotationalDrivebase drivebase, XboxController xbox, double maxRate) {
        this(drivebase, null, xbox, maxRate);
    }

    /**
     * Controls the rotational velocity of the drivebase with a joystick.
     * 
     * The maximum rotational velocity defaults to two revolutions per second.
     * 
     * @param drivebase The drivebase to control.
     * @param xbox      The joystick controller to use.
     */
    public RotationalDriveCommand(RotationalDrivebase drivebase, XboxController xbox) {
        this(drivebase, xbox, 4 * Math.PI);
    }

    @Override
    public void initialize() {
    }

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }

    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * maxRate;
    }

    @Override
    public void execute() {
        double drift = 0.0;
        if (localizer != null) {
            drift = -localizer.getRate() * DEG / S - omega;
        }
        double joy = applyDeadband(xbox.getRightX(), 0.1);
        omega = -scaleVelocity(joy);
        drive.setRotation(omega - 0.5 * drift);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setRotation(0);
        if (interrupted)
            new StartEndCommand(() -> {
                xbox.setRumble(RumbleType.kRightRumble, 0.5);
            }, () -> {
                xbox.setRumble(RumbleType.kRightRumble, 0);
            }).withTimeout(0.5).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}