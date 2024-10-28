package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class AutoShootByPose extends Command {
    private final Shooter m_shooter;
    private final RackPinion m_rackPinion;
    private final Drivetrain m_robotDrive;
    private Supplier<Pose2d> getPose;

    private final PIDController m_pid = new PIDController(0.125, 0.010, 0.0);
    private InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();

    private final Timer m_timer = new Timer();

    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(120.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(400.0*60.0);

    public AutoShootByPose(Shooter shooter, RackPinion rackPinion, Drivetrain robotDrive, Supplier<Pose2d> getPose) {
        m_shooter = shooter;
        m_rackPinion = rackPinion;
        m_robotDrive = robotDrive;

        this.getPose = getPose;

        m_pid.setIntegratorRange(-0.1, 0.1);

        m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kAngleTable);
        m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVeloTable);
        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        addRequirements(m_shooter, m_rackPinion);

    }

    private double pitchTableConversion(double x){
        return (x * (0.75))+22.6;
    }

    @Override
    public void initialize() {
        m_pid.reset();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        var alliance = DriverStation.getAlliance();

        Translation2d goalLocation;
        boolean feedShot = false;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            goalLocation = GoalConstants.kRedGoal;
             if (goalLocation.getDistance(getPose.get().getTranslation()) * 39.37 >= 260.0) {
                goalLocation = GoalConstants.kRedFeed;
                feedShot = true;
            } 
        } else {
            goalLocation = GoalConstants.kBlueGoal;
             if (goalLocation.getDistance(getPose.get().getTranslation()) * 39.37 >= 260.0) {
                goalLocation = GoalConstants.kBlueFeed;
                feedShot = true;
            }
        }

        goalLocation = compForMovement(goalLocation, feedShot);


        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        SmartDashboard.putNumber("Goal Angle", toGoal.getAngle().getDegrees());

        double angle = toGoal.getAngle().getRadians();

        double offset = (0.7 / 0.7854) * Math.abs(Math.asin(Math.sin(angle)));

        double goalDistance = toGoal.getDistance(new Translation2d()) * 39.37;

        offset *= -0.00385 * goalDistance + 1.69;
                
        
        m_rackPinion.setPose(m_pitchFilter.calculate(pitchTableConversion(m_pitchTable.get(goalDistance))) + offset);
        m_shooter.setVelo(m_velocityFilter.calculate(m_velocityTable.get(goalDistance)));
    }

    Translation2d compForMovement(Translation2d goalLocation, boolean feedShot) {

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double rx = m_robotDrive.getFieldRelativeSpeed().vx + m_robotDrive.getFieldRelativeAccel().ax * 0.030;
        double ry = m_robotDrive.getFieldRelativeSpeed().vy + m_robotDrive.getFieldRelativeAccel().ay * 0.030;

        double shotTime = m_timeTable.get(toGoal.getDistance(new Translation2d()));

        return new Translation2d(goalLocation.getX() - rx * shotTime, goalLocation.getY() - ry * shotTime);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setVelo(0.0);
        m_rackPinion.setPose(5.0);
    }

}