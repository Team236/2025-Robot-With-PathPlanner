// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.AutoCommands.EndDriveTrajectoryPID;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.commands.Scoring.L4_Score;
import frc.robot.commands.Targeting.FieldCentricTargetRight;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.NewFieldCentricTargetRight;
import frc.robot.commands.Targeting.NewNewFieldCentricTargetRight;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetForwardDistance;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoSequence extends SequentialCommandGroup {
  /** Creates a new Leg3Left. 
  public TestAutoSequence(Swerve s_Swerve) {
 //FOR TESTING OUT NEW FIELD CENTRIC TARGETING IN SEQUENTIAL COMMAND
    addCommands(//first try the targeting only to be sure it works
      //  new DriveFwd(s_Swerve, false ,6).withTimeout(2), 
        new NewNewFieldCentricTargetRight(s_Swerve).withTimeout(2)//,
       // new DriveReverse(s_Swerve, true, 20).withTimeout(2)
    );
  }
}
*/