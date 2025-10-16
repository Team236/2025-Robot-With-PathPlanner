// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetPoseWithLL extends Command {
  /** Creates a new ResetPoseWithLL. */

  //this command sets a new pose based on Limelight seeing an AprilTag, and based on which color alliance you are on
  //so that the robot yaw will be correct
  //for testing, alliance color can be set from the Driver Station (see “Team Station” on the Operation Tab).
  
  public Pose2d poseLL; //want to use this pose after this command, after moving with odometry
  private Swerve s_Swerve;    

  public GetPoseWithLL(Swerve s_Swerve)  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    //SmartDashboard.putBoolean("about to get LLPose in GetPoseWithLL: ", true);
    s_Swerve.getLLPose(); 
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    //Do everything just once, so do in init then return "true" in isFinished below
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putBoolean("isFinished in GetPoseWithLL: ", true);
    return true;
  }
}
