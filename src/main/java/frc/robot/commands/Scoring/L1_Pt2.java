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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1_Pt2 extends SequentialCommandGroup {
  /** Creates a new L1Shoot. */
  public L1_Pt2(Elevator elevator, CoralHold coralHold, CoralPivot coralPivot, AlgaePivot algaePivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(    
      Commands.parallel( //do in parallel - elev full down, coral pivot full retract
      new ElevMotionMagicPID(elevator, Constants.Elevator.L1_HEIGHT).withTimeout(2.3),//level1 elev =0, full down   
      new PIDToElevSafePosition(algaePivot).withTimeout(0.5),
      new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.5)
      //Full retract of coral pivot is the correct position for Level1 scoring
      ),    
        new CoralRelease(coralHold, Constants.CoralHold.L1_RELEASE_SPEED).withTimeout(2)
      );

  }
}
