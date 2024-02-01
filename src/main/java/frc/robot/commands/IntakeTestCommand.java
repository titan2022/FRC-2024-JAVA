package frc.robot.commands;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utility.Localizer;

import static frc.robot.utility.Constants.Unit.*;

public class IntakeTestCommand extends Command {
    private IntakeSubsystem intake;
    private XboxController xbox;
    private double maxRotationVelocity;
    private double maxIntakeVelocity;

    /**
     * Creates a new IntakeTestCommand.
     * 
     * @param intake  The intake to control.
     * @param xbox  The joystick controller to use.
     * @param maxRotationVelocity  The maximum velocity of the rotation of the intake subsystem, in centipercent output (1 is max output).
     * @param maxIntakeVelocity  The maximum velocity of the rotation of the intake subsystem, in centipercent output (1 is max output).
     */
    public IntakeTestCommand(IntakeSubsystem intake, XboxController xbox, double maxRotationVelocity, double maxIntakeVelocity) {
        this.intake = intake;
        this.xbox = xbox;
        this.maxRotationVelocity = maxRotationVelocity;
        this.maxIntakeVelocity = maxIntakeVelocity;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        
    }

    private static double applyDeadband(double joy, double deadband) {
        return Math.abs(joy) < deadband ? 0 : joy;
    }
    
    private double scaleRotationVelocity(double joy) {
        return joy * maxRotationVelocity; //Math.signum(joy) * joy * joy * maxVel;
    }

    private double scaleIntakeVelocity(double joy) {
        return joy * maxIntakeVelocity; //Math.signum(joy) * joy * joy * maxVel;
    }

    @Override
    public void execute() {
        double leftJoyX = applyDeadband(xbox.getLeftX(), 0.2);
        double leftJoyY = applyDeadband(-xbox.getLeftY(), 0.2);
        double rightJoyX = applyDeadband(xbox.getRightX(), 0.2);
        double rightJoyY = applyDeadband(-xbox.getRightY(), 0.2);
        intake.setIntakeVelocity(scaleIntakeVelocity(leftJoyY));
        intake.setRotationVelocity(scaleRotationVelocity(rightJoyY));
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(new Translation2d(0, 0));
        if(interrupted)
            new StartEndCommand(() -> {
                    xbox.setRumble(RumbleType.kLeftRumble, 0.5);
                }, () -> {
                    xbox.setRumble(RumbleType.kLeftRumble, 0);
                }).withTimeout(0.5).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}