package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDConstants;

public class Intake extends SubsystemBase {
    private CANSparkMax m_motor = new CANSparkMax(CanIDConstants.kIntake, MotorType.kBrushless);

    private SlewRateLimiter m_slew = new SlewRateLimiter(15.0);
    private double calc;

    public Intake() {
        m_motor.setInverted(true);
        m_motor.setSmartCurrentLimit(25);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setIdleMode(IdleMode.kBrake);

        m_motor.burnFlash();
    }

    public void run(double speed){
        calc = m_slew.calculate(speed);
        m_motor.set(calc);
    }
    public void stop(){
        calc =  0.0;      
        m_motor.stopMotor();
    }

    public Command runIntake(double speed){
        return runEnd(()->run(speed), ()->stop());
    }
    
    @Override
    public void periodic(){

        SmartDashboard.putNumber("Intake Current", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("Slew", calc);
        
    }
}