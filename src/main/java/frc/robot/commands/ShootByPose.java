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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.MathUtils;

public class ShootByPose extends Command {
    private final Shooter m_shooter;
    private final Drivetrain m_robotDrive;
    private final RackPinion m_rackPinion;
    private Supplier<Pose2d> getPose;

    private final PIDController m_pid = new PIDController(0.125, 0.010, 0.0);
    private InterpolatingDoubleTreeMap m_pitchTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_velocityTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_timeTable = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedPitch = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedVelocity = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap m_feedTime = new InterpolatingDoubleTreeMap();
    private final CommandXboxController m_controller;
    private double manualHoodValue = 5.0;
    private boolean manualHoodOverride = false;
    private double manualVelocityValue = 70.0;
    private boolean manualVelocityOverride = false;

    private final Timer m_timer = new Timer();

    private final SlewRateLimiter m_pitchFilter = new SlewRateLimiter(120.0);
    private final SlewRateLimiter m_velocityFilter = new SlewRateLimiter(400.0*60.0);

    public ShootByPose(Shooter shooter, Drivetrain robotDrive, RackPinion rackPinion, CommandXboxController controller,
            Supplier<Pose2d> getPose) {
        m_shooter = shooter;
        m_robotDrive = robotDrive;
        m_rackPinion = rackPinion;
        m_controller = controller;

        this.getPose = getPose;

        m_pid.setIntegratorRange(-0.1, 0.1);

        m_pitchTable = MathUtils.pointsToTreeMap(ShooterConstants.kAngleTable);
        m_feedPitch = MathUtils.pointsToTreeMap(ShooterConstants.kFeedPitch);
        m_velocityTable = MathUtils.pointsToTreeMap(ShooterConstants.kVeloTable);
        m_feedVelocity = MathUtils.pointsToTreeMap(ShooterConstants.kFeedVelocity);
        m_timeTable = MathUtils.pointsToTreeMap(ShooterConstants.kTimeTable);
        m_feedTime = MathUtils.pointsToTreeMap(ShooterConstants.kFeedTime);
        addRequirements(m_robotDrive, m_shooter, m_rackPinion);

    }

    private double pitchTableConversion(double x){
        return (x * (0.75))+22.6;
    }

    @Override
    public void initialize() {
        m_pid.reset();
        SmartDashboard.putBoolean("Manual Velocity Override", manualVelocityOverride);
        SmartDashboard.putNumber("Set Velocity Adjust", manualVelocityValue);

        SmartDashboard.putBoolean("Manual Hood Override", manualHoodOverride);
        SmartDashboard.putNumber("Set Hood Adjust", manualHoodValue);
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

        double desiredRot = -MathUtils.inputTransform(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeed;

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        SmartDashboard.putNumber("Goal Angle", toGoal.getAngle().getDegrees());

        double angle = toGoal.getAngle().getRadians();

        double offset = (0.7 / 0.7854) * Math.abs(Math.asin(Math.sin(angle)));

        double pidAngle = -1.0 * toGoal.getAngle().minus(getPose.get().getRotation()).getDegrees();

        double goalDistance = toGoal.getDistance(new Translation2d()) * 39.37;

        offset *= -0.00385 * goalDistance + 1.69;

        SmartDashboard.putNumber("Pitch offset", offset);

        SmartDashboard.putNumber("Pose Distance", goalDistance);
        SmartDashboard.putNumber("Shooter Angle Error", pidAngle);

        desiredRot = m_pid.calculate(pidAngle);

        manualHoodOverride = SmartDashboard.getBoolean("Manual Hood Override", false);
        manualVelocityOverride = SmartDashboard.getBoolean("Manual Velocity Override", false);

        if (manualHoodOverride && manualVelocityOverride) {
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_rackPinion.setPose(manualHoodValue);
            m_shooter.setVelo(manualVelocityValue);
        } else if (manualHoodOverride) {
            manualHoodValue = SmartDashboard.getNumber("Set Hood Adjust", 0);
            m_rackPinion.setPose(manualHoodValue);
        } else if (manualVelocityOverride) {
            manualVelocityValue = SmartDashboard.getNumber("Set Velocity Adjust", 70.0);
            m_shooter.setVelo(manualVelocityValue);
        } else {
            if (feedShot) {
                m_rackPinion.setPose(m_pitchFilter.calculate(pitchTableConversion(m_feedPitch.get(goalDistance))));
                m_shooter.setVelo(m_velocityFilter.calculate(m_feedVelocity.get(goalDistance)));
            } else {
                m_rackPinion.setPose(m_pitchFilter.calculate(pitchTableConversion(m_pitchTable.get(goalDistance))) + offset);
                m_shooter.setVelo(m_velocityFilter.calculate(m_velocityTable.get(goalDistance)));
            }
        }

        double xInput = -m_controller.getLeftY();
        double yInput = -m_controller.getLeftX();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            xInput = -xInput;
            yInput = -yInput;
        }

        double desiredTrans[] = MathUtils.inputTransform(xInput, yInput);

        double maxLinear = DriveConstants.kMaxSpeedMetersPerSecond*0.4;

        desiredTrans[0] *= maxLinear;
        desiredTrans[1] *= maxLinear;

        // double desiredRot = -MathUtils.inputTransform(m_controller.getRightX())*
        // DriveConstants.kMaxAngularSpeed;

        // Translation2d rotAdj= desiredTranslation.rotateBy(new
        // Rotation2d(-Math.PI/2.0)).times(desiredRot*0.05);

        // desiredTranslation = desiredTranslation.plus(rotAdj);

        m_robotDrive.drive(desiredTrans[0], desiredTrans[1], (desiredRot), true, true);

        /*
         * m_robotDrive.drive(m_slewX.calculate(
         * -inputTransform(m_controller.getLeftY()))
         * DriveConstants.kMaxSpeedMetersPerSecond,
         * m_slewY.calculate(
         * -inputTransform(m_controller.getLeftX()))
         * DriveConstants.kMaxSpeedMetersPerSecond,
         * m_slewRot.calculate(-inputTransform(m_controller.getRightX()))
         * DriveConstants.kMaxAngularSpeed,
         * fieldOrient);
         */

        SmartDashboard.putBoolean("DrivingByController", true);
    }

    Translation2d compForMovement(Translation2d goalLocation, boolean feedShot) {

        Translation2d toGoal = goalLocation.minus(getPose.get().getTranslation());

        double rx = m_robotDrive.getFieldRelativeSpeed().vx + m_robotDrive.getFieldRelativeAccel().ax * 0.030;
        double ry = m_robotDrive.getFieldRelativeSpeed().vy + m_robotDrive.getFieldRelativeAccel().ay * 0.030;

        double shotTime;
        if (feedShot) {
            shotTime = m_feedTime.get(toGoal.getDistance(new Translation2d()));
        } else {
            
            shotTime = m_timeTable.get(toGoal.getDistance(new Translation2d()));
        }
        return new Translation2d(goalLocation.getX() - rx * shotTime, goalLocation.getY() - ry * shotTime);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setVelo(0.0);
        /*
         * m_indexer.stop();
         * m_feeder.stop();
         */
        m_rackPinion.setPose(5.0);
    }

}