// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OrientWithLL extends SequentialCommandGroup {
  /** Creates a new OrientWithLL. */
  public OrientWithLL(Swerve s_Swerve) {
    addCommands(
      new GetPoseWithLL(s_Swerve).withTimeout(0.5),
      new ResetPoseWithLL(s_Swerve).withTimeout(0.5));
  }
}