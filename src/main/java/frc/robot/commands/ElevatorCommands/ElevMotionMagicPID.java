// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevMotionMagicPID extends Command {
    private Elevator elevator;
  private double desiredHeight; //desired height in inches
 
  public ElevMotionMagicPID(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    elevator.doMotionMagic(desiredHeight * (Constants.Elevator.ELEV_IN_TO_REV));
      }

  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
