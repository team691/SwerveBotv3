package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Chuck;

// simon named this class not me (luigi)
public class fiRing extends Command {
    private Chuck m_output;
    private final double m_timeout;
    private final Timer m_timer = new Timer();

    public fiRing(Chuck output, double timeout) {
        m_output = output;
        m_timeout = timeout;
        addRequirements(m_output);
    }

    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_output.cmdPrep();
    }
    
    public void execute() {
        m_output.cmdPrep();
        if (m_timeout >= 1.2) {
            m_output.cmdLaunch();
        }
    }

    public void end(boolean interrupted) {
        m_output.cmdStop();
    }

    public boolean isFinished() {
        return m_timer.get() >= m_timeout;
    }
}
