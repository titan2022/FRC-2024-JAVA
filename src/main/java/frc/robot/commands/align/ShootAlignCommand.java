package frc.robot.commands.align;
// package frc.robot.commands;

import static frc.robot.utility.Constants.Unit.IN;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.RotationalDrivebase;
import frc.robot.commands.drive.RotationCommand;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.TranslationalDrivebase;
import frc.robot.utility.Localizer;

public class ShootAlignCommand extends Command{
    private final ShooterSubsystem shooter;
    private final RotationalDrivebase rotationalDrivebase;
    private final Localizer localizer;
    private final XboxController xbox;
    private int side = 0;
    private boolean toggled = false;
    private static final double dz = 70.25*IN;
    private static final double g = 9.80665;

    public ShootAlignCommand(ShooterSubsystem shooter, RotationalDrivebase rotationalDrivebase, Localizer localizer, XboxController xbox){
        this.shooter = shooter;
        this.rotationalDrivebase = rotationalDrivebase;
        this.localizer = localizer;
        this.xbox = xbox;
        addRequirements(shooter);
        addRequirements(rotationalDrivebase);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(DriverStation.getAlliance().equals(Alliance.Red)){
            side = 1;
        }
    }

    /**
     * Returns two angles, first for the shooter's angle and second for the robot's.
     * @param location The speaker's distance to the shooter pivot. 
     * @param s Shooting velocity.
     * @param velocity Robot velocity.
     */
    private Pair<Rotation2d, Rotation2d> getAngles(Translation2d location, double s, Translation2d velocity){
        double a = location.getNorm();
        double v = velocity.getNorm();
        Rotation2d t1 = velocity.getAngle();
        Rotation2d t2 = location.getAngle();
        Rotation2d t3 = t2.minus(t1);
        double v_a = v*t3.getCos(), v_b = v*t3.getSin();
        double s_az = Math.sqrt(s*s - v_b*v_b);
        // This stuff could be faster if we assume a short window around just using trig
        double range = Math.PI / 2.0;
        double center = Math.PI / 4.0;
        double scale = 50.0;
        int reps = 3;
        Pair<Double, Double> min1 = new Pair<>(Double.MAX_VALUE, -1.0);
        while(reps-- > 0){
            for(double angle = center - range / 2.0; angle <= center + range / 2.0; angle += (range) / 100.0){
                double s_a = s_az*Math.cos(angle);
                double s_z = s_az*Math.sin(angle);
                double time = a/(s_a+v_a);
                double height = s_z*time - g/2*time*time;

                double a_val = -g/2.0;
                double b_val = s_z;
                double c_val = -height;
                double determinant = b_val*b_val - 4.0*a_val*c_val;
                if(determinant < 0) continue;
                double first_hit = (-b_val + Math.sqrt(b_val*b_val-4*a_val*c_val))/(2.0*a_val);

                if(first_hit + .01 < time) continue;
                double dif = Math.abs(-dz - height);
                if(dif < min1.getFirst()){
                    min1 = new Pair<>(dif, angle);
                }
            }
            range /= scale;
            center = min1.getSecond();
        }
        double s_z_val = s_az*Math.sin(center);
        double s_a_val = s_az*Math.cos(center);
        double s_b_val = -v_b;
        Rotation2d t4 = new Rotation2d(s_b_val, s_a_val);
        Rotation2d phi = t4.plus(t2).minus(new Rotation2d(Math.PI / 2.0));
        double s_ab = Math.sqrt(s_a_val*s_a_val + s_b_val*s_b_val);
        double s_x = s_ab*phi.getCos();
        double s_y = s_ab*phi.getSin();
        return new Pair<>(new Rotation2d(center), phi);
    }

    private Translation2d getLocation(){
        // Limelight stuff, temp until we get pi's working
        // TODO: do camera offset from shooter position
        Translation2d location = new Translation2d(
            SmartDashboard.getNumber("dx", 0.0), 
            SmartDashboard.getNumber("dy", 0.0)
        );
        // return localizer.getPosition();
        return location;
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(xbox.getAButtonPressed()){
            toggled = !toggled;
        }
        if(!toggled) return;
        Translation2d position = getLocation();
        Translation2d speakerPosition = localizer.getSpeakerLocation();
        SmartDashboard.putNumber("Speaker dx", speakerPosition.getX());
        SmartDashboard.putNumber("Speaker dy", speakerPosition.getY());
        double shooting_v = shooter.getShooterVelocity();
        Pair<Rotation2d, Rotation2d> angles = getAngles(speakerPosition, shooting_v, new Translation2d(0, 0));//translationalDrivebase.getVelocity());
        shooter.setRotation(angles.getFirst().getRadians());
        new RotationCommand((angles.getSecond().minus(new Rotation2d(Math.PI / 2))).times(-1), rotationalDrivebase, localizer);
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