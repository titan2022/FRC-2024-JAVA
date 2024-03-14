// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.control;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TeleShootCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    // public static final double RAMP_TIME = 1;
    // public static final double SHOOT_DURATION = 0.25;
    // public static final double SHOOTER_INDEX_SPEED = 0.6;
    // public static final double ELEVATOR_INDEX_SPEED = -0.6;
    public static final double SPEED = 1;
    public XboxController xbox;
    public ShooterSubsystem shooter;
    
    public TeleShootCommand(ShooterSubsystem shooter, XboxController xbox) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (xbox.getRightBumper()) {
        //     shooter.intake();
        // }
        // else 
        //     shooter.holdIndex();

        if (xbox.getRightTriggerAxis() > 0.05) {
            shooter.shoot(SPEED);
        } else {
            shooter.shoot(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

