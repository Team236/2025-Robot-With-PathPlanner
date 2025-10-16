package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetDfltValues extends Command {
   private Swerve s_Swerve;    
 

  public SetDfltValues(Swerve s_Swerve)  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
  }  
   
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.setDefaultValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
  }
