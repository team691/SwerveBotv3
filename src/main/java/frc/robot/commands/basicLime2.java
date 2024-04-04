package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Chuck;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class basicLime2 extends Command{
    private DriveTrain m_robotDrive;
    private final Limelight m_lime;
    private final Chuck m_chuck;
    private double xSpeed = 0.0;
    private double ySpeed = 0.0;
    private double rot = 0.0;
    boolean done = false;

    public basicLime2(DriveTrain drive, Limelight lime, Chuck chuck) {
        m_robotDrive = drive;
        m_lime = lime;
        m_chuck = chuck;
        addRequirements(lime, drive, chuck);
    }

        public void initialize() {
            m_robotDrive.zeroHeading();
            xSpeed = 0.5;
        }
      
        // When scheduled, run
        public void execute() {
          if (m_lime.PosArea() > 2.27 && m_lime.PosArea() < 2.37) {
            xSpeed = 0;
            m_robotDrive.setX();
            m_chuck.cmdPrep();
            done = true;
          }
          //m_chuck.cmdPrep();
          // Make new ChassisSpeeds object to work with module states
          ChassisSpeeds speeds;
          speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
          m_robotDrive.setModuleStates(moduleStates);
          

        }
    
        // If command ends or is interrupted, calls the method
        public void end(boolean interrupted) {
          m_chuck.cmdPrep();
          m_chuck.cmdLaunch();  
          m_chuck.stopRun();
        }
    
        // Returns the end of the scheduled command
        public boolean isFinished() {
            return (done);
        }
        
    }
