// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CanIDConstants {
    public static final int kLeftShooterMotor = 10;
    public static final int kRightShooterMotor = 9;
    public static final int kFeedMotor = 11;
    public static final int kIntake = 13;
    public static final int kWrist = 12;
    public static final int kRackPinion = 14;
    public static final int kClimber = 15;
    public static final int kIndexer = 16;
  }

  public static class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAngularSpeed = 1.5 * Math.PI; // radians per second

    public static final double kTrackWidth = 24.75 / 39.37; // Center distance in meters between right and left
                                                                // wheels on
    // robot
    public static final double kWheelBase = 24.75 / 39.37; // Center distance in meters between front and back
                                                                 // wheels on
    // robot

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // SPARK MAX CAN IDs
    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;

    public static final boolean kGyroReversed = false;
    public static final boolean kUseNEO = false;
    
    public static final int kFrontLeftTurningEncoderPort = 7; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 6; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 5; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 4; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = 2*Math.PI-3.589369535446167;// Encoder Offset in Radians
    public static final double kFrontRightOffset = 2*Math.PI-2.7827579975128174; // Encoder Offset in Radians
    public static final double kBackLeftOffset = 2*Math.PI-0.5142273902893066; // Encoder Offset in Radians
    public static final double kBackRightOffset = 2*Math.PI-2.4218785762786865; // Encoder Offset in Radians

    public static final double[] kFrontLeftTuningVals = { 0.015, 0.23, 0.15, 0 }; // {Static Gain, FeedForward,
                                                                                  // Proportional Gain, ModuleID for
                                                                                  // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.015, 0.23, 0.15, 1 }; // {Static Gain, FeedForward,
                                                                                   // Proportional Gain, ModuleID for
                                                                                   // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.015, 0.23, 0.15, 2 }; // {Static Gain, FeedForward,
                                                                                 // Proportional Gain, ModuleID for
                                                                                 // Tuning}
    public static final double[] kBackRightTuningVals = { 0.015, 0.23, 0.15, 3 }; // {Static Gain, FeedForward,
                                                                                  // Proportional Gain, ModuleID for
                                                                                  // Tuning}


    public static final double kWheelBaseRadius = 0.5
        * Math.sqrt(Math.pow(kWheelBase, 2) + Math.pow(kTrackWidth, 2));

    public static final double kMaxAcceleration = 3.0;
    // Conditions & Battery, Robot will not exceed this
    // speed in closed loop control
    public static final double kTestMaxAcceleration = 1.0;
    public static final double kTestMaxSpeedMetersPerSecond = 1.0;

    // but spinning fast is not particularly useful or driver
    // friendly
    public static final double kMaxAngularAccel = 1.5 * Math.PI; // Maximum Angular Speed desired. NOTE: Robot can
                                                                 // exceed this
    // but spinning fast is not particularly useful or driver
    // friendly

    public static final double kInnerDeadband = 0.02; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)

    public static final double[] kKeepAnglePID = { 1.00, 0, 0 }; // Defines the PID values for the keep angle PID

    private static final SwerveModuleState[] kLockedWheelsHelper = kDriveKinematics
        .toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
    public static final SwerveModuleState[] kLockedWheels = {
        new SwerveModuleState(0.0, kLockedWheelsHelper[0].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[1].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[2].angle.rotateBy(new Rotation2d(Math.PI / 2))),
        new SwerveModuleState(0.0, kLockedWheelsHelper[3].angle.rotateBy(new Rotation2d(Math.PI / 2)))
    };

    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }

  public static final class ModuleConstants {
    public static final class Drive{
      public static final double kGearRatio = (36.0/13.0)*(16.0/24.0)*(45.0/15.0);
      public static final double kWheelDiameter = .0985;
      public static final double kToMeters = (1.0/kGearRatio)*kWheelDiameter * Math.PI;
      public static final double kToRots = 1.0 / kToMeters;
      public static final double kNEOMaxRPS = 96.0;
      public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 1.0/((5767.0/60.0)*kToMeters);
    public static int currentLimit = 50;
    }
    public static final class Azimuth{
      public static final double kGearRatio = (50.0 / 12.0) * (72.0/12.0);
      public static final double kPositionFactor = 2*Math.PI;
      public static final double kVelocityFactor = kPositionFactor/60.0;
      public static final double kP = 0.35;
      public static final double rioKp = 0.8;
    public static int currentLimit = 20;

    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static double kCubic = 0.95;
    public static double kLinear = 0.05;
    public static double kDeadband = 0.02;
  }

  public static class AmpConstants{

    public static final Point2D[] kAngleTable = {
    /*new Point2D(x = distance, y = angle required)*/
    };

    public static final Point2D[] kVeloTable = {
    /*new Point2D(x = distance, y = velo required)*/
    };

  }

   public static class ShootingConstants{

    public static final Point2D[] kAngleTable = {
      new Point2D.Double(45.5, 47.77),
      new Point2D.Double(57.5, 41.76),
      new Point2D.Double(69.5, 35.75),
      new Point2D.Double(87.5, 29.60),
      new Point2D.Double(105.5, 23.80),
      new Point2D.Double(123.5, 18.80),
      new Point2D.Double(141.5, 13.80),
      new Point2D.Double(159.5, 10.90),
      new Point2D.Double(177.5, 8.90),
      new Point2D.Double(195.5, 5.50),
      new Point2D.Double(240.0, 4.50)
  };

  public static final Point2D[] kVeloTable = {
      new Point2D.Double(45.5, 40.0),
      new Point2D.Double(57.5, 42.24),
      new Point2D.Double(69.5, 44.81),
      new Point2D.Double(87.5, 48.00),
      new Point2D.Double(105.5, 50.50),
      new Point2D.Double(123.5, 57.00),
      new Point2D.Double(141.5, 68.00),
      new Point2D.Double(159.5, 70.28),
      new Point2D.Double(177.5, 75.00),
      new Point2D.Double(195.5, 75.00),
      new Point2D.Double(240.0, 75.00),

  };
  }

  public static class VisionConstants{

    public static Transform3d robotToCam = new Transform3d(new Translation3d(-1.0/39.37,9.25/39.37, 17.0/39.37), new Rotation3d(0,Math.toRadians(-20.0),0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  }

  public static final class Auto {
    public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(2.0, 0, 0), new PIDConstants(2.0, 0, 0), DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kWheelBaseRadius, new ReplanningConfig());
  }

  public static final class GoalConstants {
    public static final Translation2d kRedGoal = new Translation2d(643.23 / 39.37, 218.42 / 39.37);
    public static final Translation2d kBlueGoal = new Translation2d(8.00 / 39.37, 218.42 / 39.37);
    public static final Translation2d kRedFeed = new Translation2d(610.23 / 39.37, 280.0 / 39.37);
    public static final Translation2d kBlueFeed = new Translation2d(41.0 / 39.37, 280.0 / 39.37);
  }

  public static final class ShooterConstants {
    public static final double kAngleAdjustment = -0.2;

    public static final Point2D[] kAngleTable = {
        new Point2D.Double(45.5, (47.77 + kAngleAdjustment)),
        new Point2D.Double(57.5, (41.76 + kAngleAdjustment)),
        new Point2D.Double(69.5, (35.75 + kAngleAdjustment)),
        new Point2D.Double(87.5, (29.60 + kAngleAdjustment)),
        new Point2D.Double(105.5, (23.80 + kAngleAdjustment)),
        new Point2D.Double(123.5, (18.80 + kAngleAdjustment)),
        new Point2D.Double(141.5, (13.80 + kAngleAdjustment)),
        new Point2D.Double(159.5, (10.90 + kAngleAdjustment)),
        new Point2D.Double(177.5, (8.90 + kAngleAdjustment)),
        new Point2D.Double(195.5, (5.50 + kAngleAdjustment)),
        new Point2D.Double(240.0, (4.50 + kAngleAdjustment))
    };

    public static final Point2D[] kVeloTable = {
        new Point2D.Double(45.5, 37.23*60.0),
        new Point2D.Double(57.5, 40.24*60.0),
        new Point2D.Double(69.5, 42.81*60.0),
        new Point2D.Double(87.5, 47.00*60.0),
        new Point2D.Double(105.5, 50.00*60.0),
        new Point2D.Double(123.5, 57.00*60.0),
        new Point2D.Double(141.5, 68.00*60.0),
        new Point2D.Double(159.5, 70.28*60.0),
        new Point2D.Double(177.5, 75.00*60.0),
        new Point2D.Double(195.5, 75.00*60.0),
        new Point2D.Double(240.0, 75.00*60.0),

    };

    public static final Point2D[] kTimeTable = {
        new Point2D.Double(1.0, 0.3),
        new Point2D.Double(3.0, 0.35),
        new Point2D.Double(5.0, 0.4),
    };

    public static final Point2D[] kFeedPitch = {
        new Point2D.Double(240.0, 50.0),
        new Point2D.Double(290.0, 38.0),
        new Point2D.Double(350.0, 28.0)
    };

    public static final Point2D[] kFeedVelocity = {
        new Point2D.Double(240.0, 42.0*60.0),
        new Point2D.Double(290.0, 45.0*60.0),
        new Point2D.Double(350.0, 50.0*60.0)
    };

    public static final Point2D[] kFeedTime = {
        new Point2D.Double(6.0, 0.9),
        new Point2D.Double(8.0, 1.0),
        new Point2D.Double(10.0, 1.1)
    };

  }
  
}
