// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeHoldCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeHold;

public class AlgaeGrab extends Command {
  private AlgaeHold algaeHold;
  private double speed1, speed2;

  public AlgaeGrab(AlgaeHold algaeHold, double speed1, double speed2) {
    this.algaeHold = algaeHold;
    this.speed1 = speed1;
    this.speed2 = speed2;
    addRequirements(this.algaeHold);
  }

  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    this.algaeHold.setAlgaeHoldSpeed(speed1, speed2);
  }
  
  @Override
  public void end(boolean interrupted) {
    this.algaeHold.stopAlgaeHold();
  }
  @Override
  public boolean isFinished() {
    //if want to STOP when limit hit, could return 
    //return algaeHold.getAHoldLimit();
       return false;
  }
}