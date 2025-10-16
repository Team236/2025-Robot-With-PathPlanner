// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.AutoCommands.DriveFwdAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.AutoCommands.EndDriveTrajectoryPID;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.commands.Scoring.L4_Score_AutoLeg1;
import frc.robot.commands.Targeting.GetPoseWithLL;
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
public class Leg1Left extends SequentialCommandGroup {
  /** Creates a new Leg1Left. */
  public Leg1Left(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, CoralPivot coralPivot, CoralHold coralHold) {

  addCommands(
      //START ROBOT WITH BACK BUMPER FLUSH WITH BACK OF BLACK STARTING LINE, 84 inches from side ///95.75 from sideline

      new ParallelCommandGroup(        
          new DriveFwdAndTurn(s_Swerve, false, 72, 60.5).withTimeout(1.7), 
          new PIDToElevSafePosition(algaePivot).withTimeout(0.5),
          new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.5)  
        //***Consider moving the ElevMotionMagicPID command here, especially if using NewFieldCentricTargeting
          ),

     // new EndDriveTrajectoryPID(s_Swerve).withTimeout(0.25),
      
      new WaitCommand(0.1),

      new ParallelCommandGroup(
        new ElevMotionMagicPID(elevator, Constants.Elevator.L4_HEIGHT).withTimeout(2.6),
       
        new SequentialCommandGroup(
        new TargetSideDistance(s_Swerve, 0).withTimeout(0.9),
        new TargetForwardDistance(s_Swerve, 0).withTimeout(0.7),
       // new GetPoseWithLL(s_Swerve).withTimeout(0.3),
        new DriveSideways(s_Swerve, false, -5.45).withTimeout(1.3), //TUNE AT COMP//-4.5
       // new ResetPoseWithLL(s_Swerve).withTimeout(0.25),
    //Replace 5 commands above with commands below, if using field centric targeting in Leg1
    //  new WaitCommand(0.5);
    //  new NewFieldCentricTargetRight(s_Swerve).withTimeout(1.5),
       new EndDriveTrajectoryPID(s_Swerve).withTimeout(0.25)
       )
      ),
    
      //Use AutoLeg1 score, which does not bring elevator down - if bring it down at start of leg2
     new L4_Score_AutoLeg1(elevator, coralHold, coralPivot, algaePivot)


      //OTHERWISE USE:
      // new L4_Score(elevator, coralHold, coralPivot, algaePivot)   
    
  );    

  }
}
