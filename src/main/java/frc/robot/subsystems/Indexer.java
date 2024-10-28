package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Indexer extends SubsystemBase{
    //neo 550
    private CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kIndexer, MotorType.kBrushless);

    public Indexer(){
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

    public Command 
    runIndexer(double speed){
        return runEnd(()->run(speed), ()->stop());
    }
}
