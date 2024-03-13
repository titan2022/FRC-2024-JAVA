// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import frc.robot.utility.Localizer;
import frc.robot.utility.Constants.Unit;
import frc.robot.utility.Constants.Unit.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TeleRotateShooterCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double INDEX_SPEED = 0.5;
    public XboxController xbox;
    public ShooterSubsystem shooter;
    public Localizer localizer;
    public Rotation2d angle;
    
    public TeleRotateShooterCommand(ShooterSubsystem shooter, XboxController xbox, Localizer localizer) {
        this.xbox = xbox;
        this.shooter = shooter;
        this.localizer = localizer;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Methods return if reached desired location
        if (xbox.getBButton())
            shooter.setRotation(angle);
        else 
            shooter.holdAngle();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.holdAngle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}