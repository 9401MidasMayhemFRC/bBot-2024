package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class RackPinion extends SubsystemBase {
    //neo
    private CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kRackPinion, MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_motor.getEncoder();
    private SparkPIDController m_PID = m_motor.getPIDController();

    private double m_pose = 5.0;
    private boolean m_enabled = true;

    public RackPinion(){

        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(60);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setInverted(true);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) 65.0);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) 1.0);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_PID.setP(0.5);
        m_motor.burnFlash();
    }
    
    public void enable(){
        m_enabled = true;
    }
    
    public Command enableCMD(){
        return new InstantCommand(()-> enableCMD());
    }
    
    public void disable(){
        m_enabled = false;
    }
    public Command disableCMD(){
        return new InstantCommand(()-> disableCMD());
    }
    public void setPose(double position){
        m_pose = position;
    }
    
    public Command setPositionCMD(double position){
        return runOnce(()-> setPose(position));
    }

    public double getActualPosition(){
        return m_encoder.getPosition();
    }

    public double getTargetPosition(){
        return m_pose;
    }
    
    public double getCurrent(){
       return m_motor.getOutputCurrent(); 
    }

    public void zero(){
        m_enabled = false;
        m_motor.set(-0.1);
    }

    public void setZero(){
        m_encoder.setPosition(-1.0);
    }

    public void stop(){
        m_enabled = false;
        m_motor.stopMotor();
    }
    
    
    
    @Override
    public void periodic() {
        if (m_enabled) {
            m_PID.setReference(m_pose, ControlType.kPosition);
        } else {
            m_motor.stopMotor();
        }
        
        SmartDashboard.putBoolean("RackPinion Enabled", m_enabled);
        SmartDashboard.putNumber("RackPinion Actual Pose", getActualPosition());
        SmartDashboard.putNumber("RackPinion Target Pose", getTargetPosition());
        SmartDashboard.putNumber("Rackpinion Output Current", getCurrent());


        
    }

    
    
}
