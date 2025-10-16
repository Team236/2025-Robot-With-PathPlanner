// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralPivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualCoralPivot extends Command {

  private CoralPivot coralPivot;
  private double coralSpeed;
  public boolean isInitialExtend;

  /** Creates a new ManualMove. */
  public ManualCoralPivot(CoralPivot coralPivot, double coralSpeed) {
    this.coralPivot = coralPivot;
    this.coralSpeed = coralSpeed;
    addRequirements(this.coralPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //coralPivot.setIsInitialExtend(true);
    //SmartDashboard.putBoolean("Manual coral pivot command initualized", true);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 // SmartDashboard.putBoolean(("executing setCPspeed in ManualCP: "), true);
  coralPivot.setCoralPivotSpeed(coralSpeed);
 // SmartDashboard.putBoolean(("setting initial extend state to false in CP: "), true);
 // coralPivot.setIsInitialExtend(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralPivot.stopCoralPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}