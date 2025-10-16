// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeHoldCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHold;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRelease extends Command {
  private AlgaeHold algaeHold;
  private double speed;

  /** Creates a new AlgaeRelease. */
  public AlgaeRelease(AlgaeHold algaeHold, double speed){
    this.algaeHold = algaeHold;
    this.speed = speed;
    addRequirements(this.algaeHold);
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    this.algaeHold.setAlgaeReleaseSpeed(speed);
  }
  
  @Override
  public void end(boolean interrupted) {
    this.algaeHold.stopAlgaeHold();
  }
  @Override
  public boolean isFinished() {
       return false;
  }
}