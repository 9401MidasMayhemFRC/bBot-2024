// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Auto;
import frc.robot.commands.AutoShootByPose;
import frc.robot.commands.DriveByController;
import frc.robot.commands.ShootByPose;
import frc.robot.commands.ZeroClimber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.RackPinion;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Drivetrain m_drive = new Drivetrain();
  private Intake m_intake = new Intake();
  private Wrist m_wrist = new Wrist();
  private Indexer m_indexer = new Indexer();
  private Feeder m_feeder = new Feeder();
  private RackPinion m_rackPinion = new RackPinion();
  private Shooter m_shooter = new Shooter();
  private Climber m_climber = new Climber();
  private PhotonCamera m_cam = new PhotonCamera("camera");

  private PoseEstimator m_poseEstimator = new PoseEstimator(m_drive, m_cam);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final DriveByController m_driveCommand = new DriveByController(m_drive, m_driverController);

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private void configureAutoEvents() {

    // NamedCommands.registerCommand("Name of Command", actual command);
    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(m_wrist.actuateIntake(13.4), m_intake.runIntake(0.8), m_feeder.runFeeder(0.9), m_indexer.runIndexer(0.9)));
    NamedCommands.registerCommand("Kickback", (m_feeder.runFeeder(-0.4).alongWith(m_shooter.runShooter(-500.0))).withTimeout(0.050));
    NamedCommands.registerCommand("Aim", new AutoShootByPose(m_shooter,m_rackPinion,m_drive,m_poseEstimator::getPose));
    NamedCommands.registerCommand("Shoot", (m_feeder.runFeeder(0.8).alongWith(m_indexer.runIndexer(0.8))).withTimeout(0.40));
    NamedCommands.registerCommand("Mid Drive Shoot", new InstantCommand(()-> m_feeder.runFeeder(0.8)).alongWith(m_indexer.runIndexer(0.8)));
    NamedCommands.registerCommand("Stop Shoot", new InstantCommand(()-> m_feeder.stop()).alongWith(new InstantCommand(()->m_indexer.stop())));

    NamedCommands.registerCommand("Zero Climber", new ZeroClimber(m_climber));
  }

  public void configureAutoBuilder() {
    AutoBuilder.configureHolonomic(m_poseEstimator::getPose,
        m_poseEstimator::resetOdometry,
        m_drive::getChassisSpeed,
        m_drive::drive,
        Auto.autoConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Red);
          }
          return false;
        }, m_drive);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drive.setDefaultCommand(m_driveCommand);
    // Configure the trigger bindings
    configureAutoBuilder();
    configureBindings();
    configureAutoEvents();
    configureAutoChooser();
  }

  private void configureAutoChooser() {
    m_chooser = AutoBuilder.buildAutoChooser("Do Nothing");

    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.pov(180).onTrue(new InstantCommand(
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            m_drive.resetOdometry(new Pose2d(new Translation2d(15.20, 5.56), new Rotation2d()));
          } else {
            m_drive.resetOdometry(new Pose2d(new Translation2d(1.34, 5.57), new Rotation2d(Math.PI)));
          }
        }));
    m_driverController.pov(0).onTrue(new InstantCommand(
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            m_drive.resetOdometry(new Pose2d(new Translation2d(15.20, 5.56), new Rotation2d(Math.PI)));
          } else {
            m_drive.resetOdometry(new Pose2d(new Translation2d(1.34, 5.57), new Rotation2d(0)));
          }
        }));

    m_driverController.leftBumper().whileTrue(m_rackPinion.setPositionCMD(45.0).alongWith(m_shooter.setVeloCMD(-1500.0)).alongWith(new InstantCommand(()-> m_feeder.run(-0.2))))
      .onFalse(m_rackPinion.setPositionCMD(5.0).alongWith(m_shooter.setVeloCMD(0.0)).alongWith(new InstantCommand(()-> m_feeder.stop())));

    /*m_driverController.leftBumper().whileTrue(
        m_wrist.actuateIntake(13.4)
            .alongWith(m_intake.runIntake(0.8))
            .alongWith(m_indexer.runIndexer(0.9))
            .alongWith(m_feeder.runFeeder(0.9)))
        .onFalse((m_feeder.runFeeder(-0.4).alongWith(m_shooter.runShooter(-500.0))).withTimeout(0.050));*/
    m_driverController.rightBumper().whileTrue(m_wrist.actuateIntake(8.0).alongWith(m_intake.runIntake(-0.5))
        .alongWith(m_indexer.runIndexer(-0.5)).alongWith(m_feeder.runFeeder(-0.5)));

    m_driverController.leftTrigger().whileTrue(new ShootByPose(m_shooter, m_drive, m_rackPinion, m_driverController, m_poseEstimator::getPose));
    m_driverController.rightTrigger().whileTrue(m_feeder.runFeeder(0.8).alongWith(m_indexer.runIndexer(0.8)));

    m_driverController.y().onTrue(new ZeroClimber(m_climber));

    m_driverController.start().whileTrue(m_climber.retract());

    m_driverController.back().whileTrue(m_climber.extendFully());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public void teleopInit(){
    m_poseEstimator.teleopInit();
  }

  public Command getTestCommand(){
    return new ParallelCommandGroup(new ZeroClimber(m_climber));
  }
}