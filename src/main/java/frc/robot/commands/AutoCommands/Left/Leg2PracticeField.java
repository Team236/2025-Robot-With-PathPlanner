// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.EndDriveTrajectoryPID;
import frc.robot.commands.CoralHoldCommands.CoralGrabWithCounter;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg2PracticeField extends SequentialCommandGroup {
  /** Creates a new Leg2Left. */
  public Leg2PracticeField(Swerve s_Swerve, Elevator elevator) {
 //MAKE ALL Y DISTANCES AND ALL ANGLES OPPOSITE TO Right
 addCommands(
  new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(1.5),
   new DriveFwdAndSideAndTurn(s_Swerve, true, -6, -4, 0).withTimeout(3), //no turn at first
   new DriveFwdAndSideAndTurn(s_Swerve, false, 2, 2, 0).withTimeout(3),
   new EndDriveTrajectoryPID(s_Swerve).withTimeout(0.5)
   );
  }
}
   
