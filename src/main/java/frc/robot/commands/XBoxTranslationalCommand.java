package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.TranslationalDrivebase;

import static frc.robot.utility.Constants.Unit.DEG;
import static frc.robot.utility.Constants.Unit.RAD;

public class XBoxTranslationalCommand extends CommandBase {
    private TranslationalDrivebase drive;
    private XboxController controller;
    private WPI_Pigeon2 gyro;
    private double maxVel;
    private boolean isFieldOriented = true;
    private Rotation2d phiOffset = new Rotation2d();

    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param localizer  The gyroscope to use for orientation correction.
     * @param velocity  The translational velocity represented by moving the
     *  translational joystick all the way towards a cardinal direction, in
     *  meters per second.
     */
    public XBoxTranslationalCommand(TranslationalDrivebase drivebase, XboxController xbox, WPI_Pigeon2 pigeon, double velocity) {
        drive = drivebase;
        controller = xbox;
        gyro = pigeon;
        maxVel = velocity;
        addRequirements(drivebase);
    }
    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param velocity  The translational velocity represented by moving the
     *  translational joystick all the way towards a cardinal direction, in
     *  meters per second.
     */
    public XBoxTranslationalCommand(TranslationalDrivebase drivebase, XboxController xbox, double velocity) {
        this(drivebase, xbox, null, velocity);
    }
    /**
     * Creates a new TranlationalDriveCommand with optional field orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     * @param localizer  The gyroscope to use for orientation correction.
     */
    public XBoxTranslationalCommand(TranslationalDrivebase drivebase, XboxController xbox, WPI_Pigeon2 pigeon) {
        this(drivebase, xbox, pigeon, 10.);
    }
    /**
     * Creates a new TranlationalDriveCommand using intrinsic orientation.
     * 
     * The maximum velocity in a cardinal direction defaults to 5 meters per second.
     * 
     * @param drivebase  The drivebase to control.
     * @param xbox  The joystick controller to use.
     */
    public XBoxTranslationalCommand(TranslationalDrivebase drivebase, XboxController xbox) {
        this(drivebase, xbox, null, 5.);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
        SmartDashboard.putNumber("maxVel", maxVel);
        phiOffset = gyro.getRotation2d().plus(new Rotation2d(Math.PI/2));
    }

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }
    
    private double scaleVelocity(double joy) {
        return Math.signum(joy) * joy * joy * maxVel;
    }

    @Override
    public void execute() {
        if(controller.getYButtonPressed()){
            maxVel = maxVel == 10 ? 1 : 10;
            SmartDashboard.putNumber("maxVel", maxVel);
        }
        if(controller.getBButtonPressed()){
            isFieldOriented = !isFieldOriented;
            SmartDashboard.putBoolean("isFieldOriented", isFieldOriented);
        }
        if(controller.getAButtonPressed())
            phiOffset = gyro.getRotation2d().minus(new Rotation2d(Math.PI/2));
        double joyX = applyDeadband(controller.getLeftX(), 0.1);
        double joyY = applyDeadband(-controller.getLeftY(), 0.1);
        double dpadX = controller.getPOV() == -1 ? 0 : Math.sin(controller.getPOV() * DEG / RAD);
        double dpadY = controller.getPOV() == -1 ? 0 : Math.cos(controller.getPOV() * DEG / RAD);
        //SmartDashboard.putNumber("joyX", joyX);
        //SmartDashboard.putNumber("joyY", joyY);
        Translation2d velocity = new Translation2d(
            scaleVelocity(joyX) + 0.5 * dpadX,
            scaleVelocity(joyY) + 0.5 * dpadY);
        //SmartDashboard.putNumber("fieldX", velocity.getX());
        //SmartDashboard.putNumber("fieldY", velocity.getY());
        if(isFieldOriented){
            Rotation2d heading = new Rotation2d(Math.PI/2).minus(gyro.getRotation2d().minus(phiOffset));
            //SmartDashboard.putNumber("heading", heading.getDegrees());
            velocity = velocity.rotateBy(heading);
        }
        //SmartDashboard.putNumber("robotX", velocity.getX());
        //SmartDashboard.putNumber("robotY", velocity.getY());
        drive.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setVelocity(new Translation2d(0, 0));
        if(interrupted)
            new StartEndCommand(() -> {
                    controller.setRumble(RumbleType.kLeftRumble, 0.5);
                }, () -> {
                    controller.setRumble(RumbleType.kLeftRumble, 0);
                }).withTimeout(0.5).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}