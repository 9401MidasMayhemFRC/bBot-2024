// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants.Azimuth;
import frc.robot.Constants.ModuleConstants.Drive;

public class SwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_azimuthEnc;
  private final SparkPIDController m_azimuthPID;

  private final SparkPIDController m_drivingPIDController;

  private double m_referenceAngleRadians = 0;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, int turnEncoder, double encoderOffest) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    m_drivingSparkMax.setInverted(true);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    //m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use wiPth WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(Drive.kToMeters);
    m_drivingEncoder.setVelocityConversionFactor(Drive.kToMeters/60.0);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(Drive.kP);
    m_drivingPIDController.setI(Drive.kI);
    m_drivingPIDController.setD(Drive.kD);
    m_drivingPIDController.setFF(Drive.kFF);

    m_drivingSparkMax.setIdleMode(IdleMode.kBrake);
    m_turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_drivingSparkMax.setSmartCurrentLimit(Drive.currentLimit);
    m_turningSparkMax.setSmartCurrentLimit(Azimuth.currentLimit);

    m_azimuthEnc = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_azimuthEnc.setPositionConversionFactor(Azimuth.kPositionFactor);
    m_azimuthEnc.setVelocityConversionFactor(Azimuth.kVelocityFactor);
        m_azimuthEnc.setZeroOffset(encoderOffest);


    m_azimuthEnc.setInverted(true);

    m_azimuthPID = m_turningSparkMax.getPIDController();
    m_azimuthPID.setFeedbackDevice(m_azimuthEnc);
    m_azimuthPID.setP(Azimuth.kP);
    m_azimuthPID.setPositionPIDWrappingEnabled(true);
    m_azimuthPID.setPositionPIDWrappingMinInput(0.0);
    m_azimuthPID.setPositionPIDWrappingMaxInput(2*Math.PI);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_desiredState.angle = new Rotation2d(m_azimuthEnc.getPosition());
    m_drivingEncoder.setPosition(0);
  }



  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(getStateAngle()));
  }

  /**
     * Sets the reference angle for the azimuth.
     *
     * @param referenceAngleRadians Desired reference angle.
     */
    public void setReferenceAngle(double referenceAngleRadians) {
      double currentAngleRadians = m_azimuthEnc.getPosition();

      double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
      if (currentAngleRadiansMod < 0.0) {
          currentAngleRadiansMod += 2.0 * Math.PI;
      }

      // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
      // that
      double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
      if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
          adjustedReferenceAngleRadians -= 2.0 * Math.PI;
      } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
          adjustedReferenceAngleRadians += 2.0 * Math.PI;
      }

      m_referenceAngleRadians = referenceAngleRadians;
      m_azimuthPID.setReference(adjustedReferenceAngleRadians, ControlType.kPosition);
  }

  /**
   * Gets the reference angle for the azimuth.
   *
   * @return reference angle.
   */
  public double getReferenceAngle() {
      return m_referenceAngleRadians;
  }

  /*public double getStateAngle() {
      double motorAngleRadians = m_azimuthEnc.getPosition();
      motorAngleRadians %= 2.0 * Math.PI;
      if (motorAngleRadians < 0.0) {
          motorAngleRadians += 2.0 * Math.PI;
      }
      return motorAngleRadians;
  }*/

   public double getStateAngle(){
    return m_azimuthEnc.getPosition();
   }


  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(getStateAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getStateAngle()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    //double pidOutput = m_turningPIDController.calculate(optimizedDesiredState.angle.getRadians(), m_turningEncoder.getAbsolutePosition()*2*Math.PI);
    m_azimuthPID.setReference(state.angle.getRadians(),ControlType.kPosition);
    //m_desiredState = desiredState;
  
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}

