package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
//import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;


public class Wrist extends SubsystemBase{
    private final CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kWrist, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkPIDController m_PID = m_motor.getPIDController();

    private double m_position = 0.2;
    
    public Wrist(){
        //m_motor.getEncoder().setPosition(0.0);
        m_motor.setSmartCurrentLimit(40);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) 0.0);
        m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)14.0);
        m_motor.setInverted(false);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_PID.setP(0.05);
        m_motor.burnFlash();
    }

    public void setPosition(double position){
        if(position > 13.4){position = 13.0;}
        if(position < 1.0){position = 1.0;}
        m_position = position;
    }

    public Command actuateIntake(double pose){
        return runEnd(()->setPosition(pose),()->setPosition(1.0));
    }

    public double getCurrent(){
        return m_motor.getOutputCurrent();
    }

    public double getActualPosition(){
        return m_encoder.getPosition();
    }

    public double getTargetPosition(){
        return m_position;
    }


    @Override
    public void periodic(){


            m_PID.setReference(m_position, ControlType.kPosition);
        
        SmartDashboard.putNumber("Wrist Actual Pose", getActualPosition());
        SmartDashboard.putNumber("Wrist Target Pose", getTargetPosition());

    }
}
