// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaeHoldCommands.AlgaeRelease;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Algae_Score_Net extends SequentialCommandGroup {
  /** Creates a new Algae_Score. */
  public Algae_Score_Net(Elevator elevator, AlgaeHold algaeHold, AlgaePivot algaePivot) {
  addCommands(
    new PIDToElevSafePosition(algaePivot).withTimeout(1),
    new ElevMotionMagicPID(elevator, Constants.Elevator.SCORE_ALGAE_NET_HEIGHT).withTimeout(5),
    new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_SCORE_NET).withTimeout(3),
    new AlgaeRelease(algaeHold, Constants.AlgaeHold.RELEASE_SPEED).withTimeout(2),
    new PIDToElevSafePosition(algaePivot).withTimeout(3),
    new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT)
    );
  }
}
