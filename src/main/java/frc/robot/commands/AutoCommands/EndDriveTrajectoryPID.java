// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class EndDriveTrajectoryPID extends Command {
  private Swerve s_Swerve;
  private Translation2d translation2d;


  public EndDriveTrajectoryPID(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
  
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    s_Swerve.drive(
        new Translation2d(0, 0), 
        0 , 
        true,  //true for robot centric
        true //true for open loop (?)
    );
  }
  
  @Override
  public void execute() {}
  
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(
        new Translation2d(0, 0), 
        0 , 
        true,  //true for robot centric
        true //true for open loop (?)
        );
  }
  @Override
  public boolean isFinished() {
       return true;
  }
}