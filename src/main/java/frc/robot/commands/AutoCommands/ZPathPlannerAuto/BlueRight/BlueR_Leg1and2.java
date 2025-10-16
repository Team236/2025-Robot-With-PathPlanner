// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueRight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueR_Leg1and2 extends SequentialCommandGroup {
  /** Creates a new RRight_FullRun. */
  public BlueR_Leg1and2(Swerve s_Swerve) {
    addCommands(
      new BlueRLeg1(s_Swerve, false),//TODO - make true if going negative in X direction
      new BlueRLeg2(s_Swerve, false)//TODO - make true if going negative in X direction
    );
  }
}


