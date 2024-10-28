package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Shooter extends SubsystemBase{
    //Left Neo motor and encoder construction
    private CANSparkMax m_leftMotor = new CANSparkMax(CanIDConstants.kLeftShooterMotor, MotorType.kBrushless);
    private RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
    private SparkPIDController m_leftPID = m_leftMotor.getPIDController(); 
    
    //Right Neo motor and encoder construction
    private CANSparkMax m_rightMotor = new CANSparkMax(CanIDConstants.kRightShooterMotor, MotorType.kBrushless);
    private RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
    private SparkPIDController m_rightPID = m_rightMotor.getPIDController();

    private boolean m_enabled = false;
    private double m_velo = 5;
    
    public Shooter() {
        m_leftMotor.setInverted(false);
        m_leftMotor.setSmartCurrentLimit(60);
        m_leftMotor.enableVoltageCompensation(12.0);
        m_leftMotor.setIdleMode(IdleMode.kBrake);

        m_leftPID.setFeedbackDevice(m_leftEncoder);
        m_leftPID.setP(0.0001);
        m_leftPID.setI(0.0);
        m_leftPID.setD(0.0);
        m_leftPID.setFF(1.0/5676.0);

        m_leftMotor.burnFlash();
        
        m_rightMotor.setInverted(true);
        m_rightMotor.setSmartCurrentLimit(60);
        m_rightMotor.enableVoltageCompensation(12.0);
        m_leftMotor.setIdleMode(IdleMode.kBrake);
        
        m_rightPID.setFeedbackDevice(m_rightEncoder);
        m_rightPID.setP(0.0001);
        m_rightPID.setI(0.0);
        m_rightPID.setD(0.0);
        m_rightPID.setFF(1.0/5676.0);

        m_rightMotor.burnFlash();
    
    
    }

    public void enable(){
        m_enabled = true;
    }

    public Command enableCMD(){
        return new InstantCommand(()-> enable());
    }

    public void disable(){
        m_enabled = false;
    }

    public Command disableCMD(){
        return new InstantCommand(()-> disable());
    }

    public void setVelo(double velo){
        m_velo = velo;
        m_enabled = true;
    }

    public Command setVeloCMD(double velo){
        return new InstantCommand(()-> setVelo(velo));
    }

    public Command runShooter(double velo){
        return runEnd(()->setVelo(velo), ()->stop());
    }

    public double getTargetVelo(){
        return m_velo;
    }

    private double getLeftActualVelo(){
        return m_leftEncoder.getVelocity();
    }

    private double getRightActualVelo(){
        return m_rightEncoder.getVelocity();
    }

    public double getAVGActualVelo(){
        return ((getLeftActualVelo() + getRightActualVelo())/2);
    }

    public void stop(){
m_enabled = false;
    }

    @Override
    public void periodic() {

        if(m_enabled){
            m_leftPID.setReference(m_velo, ControlType.kVelocity);
            m_rightPID.setReference(m_velo, ControlType.kVelocity);
        } else {
            m_leftMotor.stopMotor();
            m_rightMotor.stopMotor();
        }

        SmartDashboard.putBoolean("Shooter Enabled", m_enabled);
        SmartDashboard.putNumber("Shooter Target Velo.", m_velo);
        SmartDashboard.putNumber("Shooter Actual Left Velo.", getLeftActualVelo());

        
    }
}