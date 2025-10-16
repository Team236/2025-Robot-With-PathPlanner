package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommands.EndDriveTrajectoryPID;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ARightBranch extends SequentialCommandGroup {
    //Try this command to be able to target while driving
    //but driver must stop driving pretty quickly after this button is pressed
    public ARightBranch(Swerve s_Swerve) {
       addCommands( 
            new EndDriveTrajectoryPID(s_Swerve).withTimeout(0.5),
            new WaitCommand(0.5),
            new FieldCentricTargetRight(s_Swerve)
       );           
  
    }
  }
  