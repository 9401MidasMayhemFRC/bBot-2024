// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.*;
import frc.robot.utilities.ChassisAccel;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;

/**
 * Implements a swerve Drivetrain Subsystem for the Robot
 */
public class Drivetrain extends SubsystemBase {

  private double keepAngle = 0.0; // Double to store the current target keepAngle in radians
  private double timeSinceRot = 0.0; // Double to store the time since last rotation command
  private double lastRotTime = 0.0; // Double to store the time of the last rotation command
  private double timeSinceDrive = 0.0; // Double to store the time since last translation command
  private double lastDriveTime = 0.0; // Double to store the time of the last translation command
  private double scaleFactor = 1.0;

  // Create the PIDController for the Keep Angle PID
  private final PIDController m_keepAnglePID = new PIDController(DriveConstants.kKeepAnglePID[0],
      DriveConstants.kKeepAnglePID[1], DriveConstants.kKeepAnglePID[2]);

  private final Timer keepAngleTimer = new Timer(); // Creates timer used in the perform keep angle function

  private SlewRateLimiter m_slewX = new SlewRateLimiter(12.0);
  private SlewRateLimiter m_slewY = new SlewRateLimiter(12.0);
  private SlewRateLimiter m_slewRot = new SlewRateLimiter(32.0);


  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

    // Create MAXSwerveModules
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,DriveConstants.kFrontLeftTurningEncoderPort,DriveConstants.kFrontLeftOffset);
  
    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightTurningEncoderPort,DriveConstants.kFrontRightOffset);
  
    private final SwerveModule m_rearLeft = new SwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftTurningEncoderPort,DriveConstants.kBackLeftOffset);
  
    private final SwerveModule m_rearRight = new SwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightTurningEncoderPort,DriveConstants.kBackRightOffset);

  // Creates an ahrs gyro (NavX) on the MXP port of the RoboRIO
  private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  // Creates Odometry object to store the pose of the robot
  private final SwerveDriveOdometry m_odometry 
    = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyro(), getModulePositions());

  private final SwerveDriveOdometry m_autoOdometry 
    = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyro(), getModulePositions());

  private ChassisSpeeds m_lastDriveSpeed = new ChassisSpeeds();
  private ChassisAccel m_driveAccel = new ChassisAccel();
  
  private final double[] m_latestSlew = {0.0,0.0,0.0};

  private SwerveModuleState[] m_desStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));
  
  /**
   * Constructs a Drivetrain and resets the Gyro and Keep Angle parameters
   */
  public Drivetrain() {
    keepAngleTimer.reset();
    keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    m_odometry.resetPosition(getGyro(), getModulePositions(), new Pose2d());
    ahrs.reset();

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean keepAngle) {

     xSpeed *= scaleFactor;
     ySpeed *= scaleFactor;
     rot *= scaleFactor;   

    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);  
    rot = m_slewRot.calculate(rot);

    m_latestSlew[0] = xSpeed;
    m_latestSlew[1] = ySpeed;
    m_latestSlew[2] = rot;
    
    if(keepAngle){
      rot = performKeepAngle(xSpeed, ySpeed, rot); // Calls the keep angle function to update the keep angle or rotate
    }
    
    if(fieldRelative){
      setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyro()));
    }
    else{
      setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
    // All other subsystem initialization
    // ...

    
  }




  @Override
  public void periodic() {

    m_driveAccel = new ChassisAccel(getChassisSpeed(),m_lastDriveSpeed,GlobalConstants.kLoopTime);

    m_lastDriveSpeed = getChassisSpeed();

    m_fieldRelVel = new FieldRelativeSpeed(getChassisSpeed(), getGyro());
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, GlobalConstants.kLoopTime);
    m_lastFieldRelVel = m_fieldRelVel;

    double xSpeed = getChassisSpeed().vxMetersPerSecond;
    double ySpeed = getChassisSpeed().vyMetersPerSecond;

    double speed = Math.sqrt(xSpeed*xSpeed+ySpeed*ySpeed);

    SmartDashboard.putNumber("Speed", speed);
    //SmartDashboard.putNumber("Tilt", getTilt());

    // SmartDashboard.putNumber("Accel X", m_fieldRelAccel.ax);
    // SmartDashboard.putNumber("Accel Y", m_fieldRelAccel.ay);
    // SmartDashboard.putNumber("Alpha", m_fieldRelAccel.alpha);

     SmartDashboard.putNumber("Front Left Speed", m_frontLeft.getState().speedMetersPerSecond);
     SmartDashboard.putNumber("Front Right Speed",m_frontRight.getState().speedMetersPerSecond);
     SmartDashboard.putNumber("Back Left Speed", m_rearLeft.getState().speedMetersPerSecond);
     SmartDashboard.putNumber("Back Right Speed", m_rearRight.getState().speedMetersPerSecond);

     SmartDashboard.putNumber("Front Left Encoder", m_frontLeft.getState().angle.getRadians());
     SmartDashboard.putNumber("Front Right Encoder", m_frontRight.getState().angle.getRadians());
     SmartDashboard.putNumber("Rear Left Encoder", m_rearLeft.getState().angle.getRadians());
     SmartDashboard.putNumber("Rear Right Encoder", m_rearRight.getState().angle.getRadians());

     //SmartDashboard.putNumber("Front Left Speed", m_frontLeft.getState().speedMetersPerSecond);
     //SmartDashboard.putNumber("Front Right Speed",m_frontRight.getState().speedMetersPerSecond);
     //SmartDashboard.putNumber("Back Left Speed", m_backLeft.getState().speedMetersPerSecond);
     //SmartDashboard.putNumber("Back Right Speed", m_backRight.getState().speedMetersPerSecond);
    // Update swerve drive odometry periodically so robot pose can be tracked
    updateOdometry();

    // Calls get pose function which sends the Pose information to the
    // SmartDashboard
    getPose();
  }

  public void drive(ChassisSpeeds chassisSpeeds){
    setModuleStates(chassisSpeeds);
  }

  public void stop(){
    m_rearLeft.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
    m_frontLeft.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0.0, new Rotation2d(0.0)));

  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public FieldRelativeSpeed getFieldRelativeSpeed() {
    return m_fieldRelVel;
  }

  public FieldRelativeAccel getFieldRelativeAccel() {
    return m_fieldRelAccel;
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(secondOrderKinematics(chassisSpeeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_desStates = desiredStates;
    SmartDashboard.putNumber("Front Left Desired", desiredStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Front Right Desired",desiredStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Left Desired", desiredStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("Back Right Desired", desiredStates[3].speedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds){
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d rotAdj= translation.rotateBy(new Rotation2d(-Math.PI/2.0)).times(chassisSpeeds.omegaRadiansPerSecond*0.045);

    translation = translation.plus(rotAdj);

    return new ChassisSpeeds(translation.getX(),translation.getY(),chassisSpeeds.omegaRadiansPerSecond);
  }
  

  public double getRoll() {
    return -1.0*ahrs.getPitch();
    // return MathUtils.pythagorean(ahrs.getRoll(), ahrs.getPitch());
  }
  public double getPitch() {
    return -1.0*ahrs.getRoll();
    // return MathUtils.pythagorean(ahrs.getRoll(), ahrs.getPitch());
  }

  public double getTiltVel() {
    return ahrs.getRawGyroY();
  }

  /**
   * Updates odometry for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */  
  public void updateOdometry() {
    m_odometry.update(getGyro(), getModulePositions());
  }

  public void updateAutoOdometry() {
    m_autoOdometry.update(getGyro(), getModulePositions());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * 
   * @return Rotation2d object containing Gyro angle
   */
  public Rotation2d getGyro() {
    return ahrs.getRotation2d().times(-1.0);
  }

  /**
   * Function created to retreieve and push the robot pose to the SmartDashboard
   * for diagnostics
   * 
   * @return Pose2d object containing the X and Y position and the heading of the
   *         robot.
   */
  public Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    Translation2d position = pose.getTranslation();

    SmartDashboard.putNumber("Robot X", position.getX());
    SmartDashboard.putNumber("Robot Y", position.getY());
    SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());
    
    SmartDashboard.putNumber("Robot Roll", getRoll());
    SmartDashboard.putNumber("Robot Pitch", getPitch());

    return pose;
  }

  public Pose2d getAutoPose() {
    updateAutoOdometry();
    //Pose2d pose = m_autoOdometry.getPoseMeters();
    //Translation2d position = pose.getTranslation();
    return m_autoOdometry.getPoseMeters();
  }

  

  /**
   * Resets the odometry and gyro to the specified pose.
   *
   * @param pose in which to set the odometry and gyro.
   */
  public void resetOdometry(Pose2d pose) {
    ahrs.reset();
    ahrs.setAngleAdjustment(pose.getRotation().times(-1.0).getDegrees());
    updateKeepAngle();
    m_odometry.resetPosition(getGyro().times(-1.0), getModulePositions(), pose);
    m_autoOdometry.resetPosition(getGyro().times(-1.0), getModulePositions(), pose);
  }

  public void setPose(Pose2d pose){
    m_odometry.resetPosition(getGyro().times(-1.0), getModulePositions(), pose);
  }



    /**
   * Resets the gyro to the given angle
   * 
   * @param angle the angle of the robot to reset to
   */
  public void resetOdometry(Rotation2d angle) {
    ahrs.reset();
    ahrs.setAngleAdjustment(angle.getDegrees());
    Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
    updateKeepAngle();
    m_odometry.resetPosition(getGyro().times(-1.0), getModulePositions(), pose);  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the
   * swerve drive kinematics.
   * 
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
   */
  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }
  
  public ChassisSpeeds getCorDesChassisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_desStates[0],m_desStates[1],m_desStates[2],m_desStates[3]);
  }

  public ChassisAccel getChassisAccel(){
    return m_driveAccel;
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_rearLeft.getPosition(),
      m_rearRight.getPosition()};
  }

    /**
   * Keep angle function is performed to combat drivetrain drift without the need
   * of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the
   * keepAngle value. This value is updated when the robot
   * is rotated manually by the driver input
   * 
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot    is the input drive rotation speed command
   */
  private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
                         // called
    if (Math.abs(rot) >= DriveConstants.kMinRotationCommand) { // If the driver commands the robot to rotate set the
                                                               // last rotate time to the current time
      lastRotTime = keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommand
        || Math.abs(ySpeed) >= DriveConstants.kMinTranslationCommand) { // if driver commands robot to translate set the
                                                                        // last drive time to the current time
      lastDriveTime = keepAngleTimer.get();
    }
    timeSinceRot = keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time
    timeSinceDrive = keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive time
    if (timeSinceRot < 0.25) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
                              // move to finish
      keepAngle = getGyro().getRadians();
    } else if (Math.abs(rot) < DriveConstants.kMinRotationCommand && timeSinceDrive < 0.25) { // Run Keep angle pid
                                                                                              // until 0.75s after drive
                                                                                              // command stops to combat
                                                                                              // decel drift
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle); // Set output command to the result of the
                                                                            // Keep Angle PID
    }
    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getGyro().getRadians();
  }

  public void setSpeedScale(double factor){
     scaleFactor = factor;
  }

  }
