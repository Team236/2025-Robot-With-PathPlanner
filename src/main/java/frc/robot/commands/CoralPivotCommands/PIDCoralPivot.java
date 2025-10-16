// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralPivotCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDCoralPivot extends Command {
  /** Creates a new PIDCoralPivot. */
  private CoralPivot coralPivot;
    
  private final PIDController pidController;
  private double kP = Constants.CoralPivot.KP;
  private double kI = Constants.CoralPivot.KI;
  private double kD = Constants.CoralPivot.KD;
  //private double revs;

  public PIDCoralPivot(CoralPivot coralPivot, double revs) {
    pidController = new PIDController(kP, kI, kD);
    this.coralPivot = coralPivot;
    //this.revs = revs;
    addRequirements(this.coralPivot);

    pidController.setSetpoint(revs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    //coralPivot.setIsInitialExtend(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // SmartDashboard.putBoolean(("executing setCPspeed in PID CP: "), true);
    coralPivot.setCoralPivotSpeed(pidController.calculate(coralPivot.getCoralEncoder()));
   // SmartDashboard.putBoolean(("setting initial extend state false PID CP: "), true);
    //coralPivot.setIsInitialExtend(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     //Removed statement below - may cause jolting from one position to another
    coralPivot.stopCoralPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}