// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetTargetingValues extends InstantCommand {
  private Swerve s_Swerve;
  private String targetingType; // "left" or "right" or "algae"

  public GetTargetingValues(Swerve s_Swerve, String targetingType) {
    this.s_Swerve = s_Swerve;
    this.targetingType = targetingType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Swerve);
    s_Swerve.updateTargetingValues(targetingType);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.updateTargetingValues(targetingType);
  }
}
