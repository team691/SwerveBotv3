// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chuck;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootbackout extends SequentialCommandGroup {
    private Chuck m_output;
    private DriveTrain m_robotDrive;
    //private final Timer m_timer = new Timer();
  
  /** Creates a new shootbackout. */
  public void shootBackout() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new fiRing(m_output, 4.0));
    addCommands(new forwardCmd(m_robotDrive, 4.0));
  }
}
