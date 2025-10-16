// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class FullRunLeft extends SequentialCommandGroup {
  /** Creates a new FullRunParallel. */
  public FullRunLeft(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(
    // MAKE ALL Y DISTANCES AND ALL ANGLES OPPOSITE TO FullRunRight 

    new Leg1Left(s_Swerve, elevator, algaePivot, coralPivot, coralHold),
    new Leg2Left(s_Swerve, coralHold, coralPivot, elevator),
    new Leg3Left(s_Swerve, elevator, algaePivot, coralPivot, coralHold) 
  
    );
  
  }
}