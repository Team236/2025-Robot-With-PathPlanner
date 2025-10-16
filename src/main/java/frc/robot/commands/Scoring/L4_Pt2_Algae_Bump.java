// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeHold;
import frc.robot.commands.AlgaeHoldCommands.AlgaeGrab;
import frc.robot.commands.AlgaeHoldCommands.AlgaeRelease;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.AutoCommands.EndDriveTrajectoryPID;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4_Pt2_Algae_Bump extends SequentialCommandGroup {
  /** Creates a new Algae_Bump_After_Score. */
  public L4_Pt2_Algae_Bump(Elevator elevator, CoralPivot coralPivot, CoralHold coralHold, AlgaePivot algaePivot, frc.robot.subsystems.AlgaeHold algaeHold, Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
            Commands.parallel( //do in parallel so elevator stays up the whole time
        new ElevMotionMagicPID(elevator, Constants.Elevator.L4_HEIGHT).withTimeout(1.4), 
             
        Commands.sequence(
        // new WaitCommand(1.2), //wait for elevator to go up
          // new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4).withTimeout(0.9),
          new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED).withTimeout(0.5),
          new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.9)
          )
          ),

/*
      Commands.sequence(    
        new ElevMotionMagicPID(elevator, Constants.Elevator.PICK_ALGAE_L3_HEIGHT).withTimeout(2.3), 
   
        // wait for elevator to go up 
        new WaitCommand(0.5),
        new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_REEF_PICKUP).withTimeout(0.5),

        Commands.parallel(
          new AlgaeGrab(algaeHold, Constants.AlgaeHold.HOLD_SPEED1, Constants.AlgaeHold.HOLD_SPEED2),
          Commands.sequence(
            new WaitCommand(1),
            new PIDToElevSafePosition(algaePivot).withTimeout(0.5)
          ) 
        )
      ) */
      
        new SequentialCommandGroup(  
          new DriveFwdAndSideAndTurn(swerve, true, 0, -3, 0).withTimeout(0.5),
          new EndDriveTrajectoryPID(swerve).withTimeout(0.25),
          new GetPoseWithLL(swerve).withTimeout(0.25),
          new ResetPoseWithLL(swerve).withTimeout(0.25)
        ),
        new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_BUMP).withTimeout(0.7),
        new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(1),
        new AlgaeRelease(algaeHold, -0.5).withTimeout(0.7),
        

      new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_ELEVATOR_SAFE_POSITION).withTimeout(0.4)
        
      
    );
  }

}
