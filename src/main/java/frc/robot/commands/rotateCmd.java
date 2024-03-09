package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveTrain;

public class rotateCmd extends Command{
    private DriveTrain m_robotDrive;
    private final double m_timeout;
    private final Timer m_timer = new Timer();

    public rotateCmd(DriveTrain drive, double timeout) {
        m_robotDrive = drive;
        m_timeout = timeout;
        addRequirements(m_robotDrive);
    }

        public void initialize() {
            m_robotDrive.zeroHeading();
            m_timer.reset();
            m_timer.start();
        }
      
        // When scheduled, run
        public void execute() {
          double xSpeed = 0.0;
          double ySpeed = 0.0;
          double rot = 1.0;
    
          // Make new ChassisSpeeds object to work with module states
          ChassisSpeeds speeds;
          speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
          m_robotDrive.setModuleStates(moduleStates);
        }
    
        // If command ends or is interrupted, calls the method
        public void end(boolean interrupted) {
          m_robotDrive.setX();
        }
    
        // Returns the end of the scheduled command
        public boolean isFinished() {
            return m_timer.get() >= m_timeout;
        }
        
    }
