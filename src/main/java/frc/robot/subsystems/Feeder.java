package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;
//neo 550
public class Feeder extends SubsystemBase {
    private CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kFeedMotor, MotorType.kBrushless);
    
    public Feeder(){
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(20);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor.burnFlash();
    }

    public void run(double speed){
        m_motor.set(speed);
    }

    public void stop(){
        m_motor.stopMotor();
    }

    public Command runFeeder(double speed){
        return runEnd(()->run(speed), ()->stop());
    }
}