package frc.robot.subsystems;

// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import frc.robot.subsystems.Lights;

// Pneumatic Climber
public class Climber extends SubsystemBase{

    // Position/Status of the Solenoid/Pneumatic (Up vs. Down)
    private DoubleSolenoid.Value state = DoubleSolenoid.Value.kReverse;
    // ID number 9 in REV
    private final PneumaticHub pHub = new PneumaticHub(9);
    // On Pneumatic Hub ports 0 and 1
    private final DoubleSolenoid sole1 = pHub.makeDoubleSolenoid(3,1);
    private final DoubleSolenoid sole2 = pHub.makeDoubleSolenoid(14,8);
    //private final Lights m_lights = new Lights();

    public void AccuateUp() {
        state = DoubleSolenoid.Value.kForward;
        sole1.set(state);
        sole2.set(state);
        //m_lights.climberTheme();
    }

    public void AcctuateDown() {
        state = DoubleSolenoid.Value.kReverse;
        sole1.set(state);
        sole2.set(state);
        //m_lights.climberTheme();
    }    
}