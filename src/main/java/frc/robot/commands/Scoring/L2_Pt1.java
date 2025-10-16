// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;

public class L2_Pt1 extends SequentialCommandGroup {
 
  public L2_Pt1(Elevator elevator, CoralHold coralHold, CoralPivot coralPivot, AlgaePivot algaePivot) {

  addCommands(
     Commands.parallel( 
     new PIDToElevSafePosition(algaePivot).withTimeout(0.5),
     new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.5)
     ),
     Commands.parallel( //do in parallel so elevator stays up the whole time
     new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL2),//.withTimeout(0.9),
     new ElevMotionMagicPID(elevator, Constants.Elevator.L2_HEIGHT)


      // Commands.sequence(
       // new WaitCommand(0.5),     //wait for elevator to go up
       
        // new CoralRelease(coralHold, Constants.CoralHold.L2_RELEASE_SPEED).withTimeout(0.5),
        // new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.9)
        // )
     )
      // new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(1.2)
    );
  }
}
