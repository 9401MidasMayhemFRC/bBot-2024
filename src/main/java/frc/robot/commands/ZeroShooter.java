package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RackPinion;

public class ZeroShooter extends Command{
    private final RackPinion m_rackPinion;
    private boolean m_finished = false;
    private Timer m_timer = new Timer();
    
    public ZeroShooter(RackPinion rackPinion){
        m_rackPinion = rackPinion;
        addRequirements(m_rackPinion);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_finished = false;
        m_rackPinion.zero();
    }

    @Override
    public void execute() {
        if ( m_timer.get() >= 0.25 && m_rackPinion.getCurrent() >= 5.0 /* untested */){
            m_rackPinion.setZero();
            m_rackPinion.stop();
            m_finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_rackPinion.enable();
        m_timer.stop();
    }
}
