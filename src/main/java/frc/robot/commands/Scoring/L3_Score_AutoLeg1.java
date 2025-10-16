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


public class L3_Score_AutoLeg1 extends SequentialCommandGroup {

  public L3_Score_AutoLeg1(Elevator elevator, CoralHold coralHold, CoralPivot coralPivot, AlgaePivot algaePivot) {

    addCommands(
      Commands.parallel(
       new PIDToElevSafePosition(algaePivot).withTimeout(0.5),
       new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.5)
       ),
       Commands.parallel( //do in parallel so elevator stays up the whole time
         new ElevMotionMagicPID(elevator, Constants.Elevator.L3_HEIGHT).withTimeout(2.3),
         Commands.sequence(
           //new WaitCommand(0.7), //wait for elevator to go up
            new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL3).withTimeout(0.9),
            new CoralRelease(coralHold, Constants.CoralHold.L3_RELEASE_SPEED).withTimeout(0.5),
            new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.9)
          )
        //NO ELEVATOR DOWN - WILL GET DONE AT START OF LEG2 FOR AUTO
       )
      );

  }
}

