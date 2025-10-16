// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.MetricDriveFwdSideTurn;
import frc.robot.subsystems.Swerve;




public class TargetPose extends InstantCommand { //Takes a pose and will move the robot there with field centric targeting stuff
  private Swerve s_Swerve;
  private Pose2d robotFieldPose;
  private Pose2d targetFieldPose;
  private double tv;
  private Optional<Alliance> alliance = DriverStation.getAlliance();

  public TargetPose(Swerve swerve, Pose2d targetFieldPose) {
    this.s_Swerve = swerve;
    this.targetFieldPose = targetFieldPose;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv == 1) {
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            } else if (alliance.get() == Alliance.Red) {
                robotFieldPose = LimelightHelpers.getBotPose2d_wpiRed("limelight");
            }

            s_Swerve.setPose(robotFieldPose);

            double x1 = robotFieldPose.getX();
            double y1 = robotFieldPose.getY();
            double angle1 = robotFieldPose.getRotation().getDegrees();

            double x2 = targetFieldPose.getX();
            double y2 = targetFieldPose.getY();
            double angle2 = targetFieldPose.getRotation().getDegrees();

            CommandScheduler.getInstance().schedule(new MetricDriveFwdSideTurn(s_Swerve, false, x1, y1, angle1, x2, y2, angle2));
        }
    }
  }
}
