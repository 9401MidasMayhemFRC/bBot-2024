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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Climber extends SubsystemBase {

    private CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kClimber, MotorType.kBrushless);
    private RelativeEncoder m_encoder =  m_motor.getEncoder();
    private SparkPIDController m_PID = m_motor.getPIDController();

    private double setpoint = 0.0;
    private boolean m_enabled = true;

    public Climber(){

        m_motor.restoreFactoryDefaults();
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setSmartCurrentLimit(60);
        m_motor.setInverted(true);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)(390.0*.75));
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.0);

        m_PID.setP(0.5);
        m_PID.setI(0.0);
        m_PID.setD(0.0);
        m_PID.setFF(1.0/5676.0);

        m_motor.burnFlash();

    }
    
    public void zero(){
        m_enabled = false;
        m_motor.set(-0.2);
    }

    public void setZero(){
        m_encoder.setPosition(0.0);
    }

    public double getCurrent(){
        return m_motor.getOutputCurrent();
    }

    public void stop(){
        m_motor.stopMotor();
        m_enabled = false;
    }

    public void run(){
        m_enabled = true;
    }
    
    public void stopZero() {
        m_motor.stopMotor();
    }

    public Command retract(){
        return startEnd(()->setpoint = 5.0, ()-> setpoint = m_encoder.getPosition());
    }

    public Command extend(){
        return startEnd(()->setpoint = m_motor.getSoftLimit(SoftLimitDirection.kForward)-(20.0*0.75), ()-> setpoint = m_encoder.getPosition());
    }

    public Command retractFully(){
        return runOnce(()-> setpoint = 2.0);
    }

    public Command extendFully(){
        return runOnce(()-> setpoint = m_motor.getSoftLimit(SoftLimitDirection.kForward)-(20.0*0.75));
    }


    @Override
    public void periodic() {

        if(m_enabled){
            m_PID.setReference(setpoint, ControlType.kPosition);
        }

        SmartDashboard.putNumber("Current of Climber", getCurrent());
        SmartDashboard.putNumber("Climber Pose.", m_encoder.getPosition());
        
    }
}
