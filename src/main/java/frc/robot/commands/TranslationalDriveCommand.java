package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

public class TranslationalDriveCommand extends Command {
    private TranslationalDrivebase drive;
    private XboxController xbox;
    private Localizer localizer;
    private double maxVel;
    private boolean isFieldOriented = false;
    private Rotation2d phiOffset = new Rotation2d(0);

    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * @param drivebase The drivebase to control.
     * @param localizer The localizer to use for orientation correction.
     * @param xbox      The joystick controller to use.
     * @param velocity  The translational velocity represented by moving the
     *                  translational joystick all the way towards a cardinal
     *                  direction, in
     *                  meters per second.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drive, Localizer localizer, XboxController xbox,
            double maxVel) {
        this.drive = drive;
        this.localizer = localizer;
        this.xbox = xbox;
        this.maxVel = maxVel;
        addRequirements(drive);
    }

    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * @param drivebase The drivebase to control.
     * @param xbox      The joystick controller to use.
     * @param velocity  The translational velocity represented by moving the
     *                  translational joystick all the way towards a cardinal
     *                  direction, in
     *                  meters per second.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox, double velocity) {
        this(drivebase, null, xbox, velocity);
    }

    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase The drivebase to control.
     * @param localizer The localizer to use for orientation correction.
     * @param xbox      The joystick controller to use.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, Localizer localizer, XboxController xbox) {
        this(drivebase, localizer, xbox, 6.0);
    }

    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase The drivebase to control.
     * @param xbox      The joystick controller to use.
     */
    public TranslationalDriveCommand(TranslationalDrivebase drivebase, XboxController xbox) {
        this(drivebase, null, xbox, 6.0);
    }

    @Override
    public void initialize() {

    }

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }

    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * maxVel;
    }

    @Override
    public void execute() {
        if (xbox.getBButtonPressed() && localizer != null) {
            isFieldOriented = !isFieldOriented;
            SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
        }

        if (xbox.getAButtonPressed()) {
            phiOffset = localizer.getHeading().plus(new Rotation2d(Math.PI / 2));
        }

        double joyX = applyDeadband(xbox.getLeftX(), 0.15);
        double joyY = applyDeadband(-xbox.getLeftY(), 0.15);
        Translation2d velocity = new Translation2d(scaleVelocity(joyX), scaleVelocity(joyY));

        if (isFieldOriented) {
            Rotation2d heading = new Rotation2d(Math.PI / 2).plus(localizer.getHeading().minus(phiOffset));
            velocity = velocity.rotateBy(heading);
        }

        drive.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVelocity(new Translation2d(0, 0));
        if (interrupted) {
            new StartEndCommand(() -> {
                xbox.setRumble(RumbleType.kLeftRumble, 0.5);
            }, () -> {
                xbox.setRumble(RumbleType.kLeftRumble, 0);
            }).withTimeout(0.5).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}