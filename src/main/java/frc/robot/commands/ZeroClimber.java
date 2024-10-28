package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends Command{

    private Climber m_climber;
    private Timer timer = new Timer();

    private boolean m_finished = false;

    
    public ZeroClimber(Climber climber){
        m_climber = climber;

        addRequirements(m_climber);

    }

    @Override
    public void initialize() {
        m_finished = false;
        timer.reset();
        timer.start();
        m_climber.zero();
    }

    @Override
    public void execute() {

        if (timer.get() >= 0.25 && m_climber.getCurrent() > 15.0){
            m_climber.setZero();
            m_climber.stopZero();
            m_finished = true;
        }
            
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.run();
        m_climber.retractFully();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
       return m_finished;
    }
}
