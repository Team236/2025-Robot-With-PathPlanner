// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueLeft;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.commands.Scoring.L4_Score_AutoLeg1;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

/* NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html 
*/
public class BlueL_FullRun extends SequentialCommandGroup {
  /** Creates a new RRight_FullRun. */
  public BlueL_FullRun(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(
      new BlueLLeg1(s_Swerve, false),//TODO - make true if going negative in X direction
      // Could use AutoLeg2 score, which does not bring elevator down - if bring it down at start of leg2
      new L4_Score_AutoLeg1(elevator, coralHold, coralPivot, algaePivot),
      
      // the prior command "L4_Score_AutoLeg1" does not bring down elevator
      // Bring elevator down while driving next leg
      Commands.parallel(
            new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(1.5),
            // drive next legs to go pickup new coral
            new BlueLLeg2(s_Swerve, false)  //TODO - make true if going negative in X direction
      ),
      new BlueLLeg3(s_Swerve, true),    //TODO - make false if going positive in X direction
      // intake the coral so that we can possibly move on to leg4

      new BlueLLeg4(s_Swerve, true),    
      new L4_Score_AutoLeg1(elevator, coralHold, coralPivot, algaePivot) );
  }
}

