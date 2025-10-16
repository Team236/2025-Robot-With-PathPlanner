// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToCoralRightLL extends InstantCommand {
  private Swerve s_Swerve;
  private Pose2d robotFieldPose;
  private Pose2d targetFieldPose;
  private double tv;
  private int targetId;
  private Optional<Alliance> alliance = DriverStation.getAlliance();

  /** Creates a new GoToCoralRightLL. */
  public GoToCoralRightLL(Swerve swerve) {
    this.s_Swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
    targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

    if (alliance.isPresent() && tv == 1 && Constants.Targeting.REEF_IDS.contains(targetId)) {
      
      if (alliance.get() == Alliance.Red) {
        robotFieldPose = LimelightHelpers.getBotPose2d_wpiRed("limelight");
        s_Swerve.setPose(robotFieldPose);
      }
      else if (alliance.get() == Alliance.Blue) {
        robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        s_Swerve.setPose(robotFieldPose);
      }

      double turnAngle = LimelightHelpers.getTargetPose_CameraSpace("limelight")[5];
      double forward = Units.metersToInches(LimelightHelpers.getTargetPose_CameraSpace("limelight")[0]) - Constants.Targeting.DIST_CAMERA_TO_BUMPER_FWD;//Constants.Targeting.DIST_TO_CENTER;
      double side = Units.metersToInches(LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);
      
      //side += Constants.Targeting.DIST_CORAL_TAG_CENTER * Math.cos(Units.degreesToRadians(turnAngle));
      side *= -1;
      turnAngle *= -1;
          //  SmartDashboard.putNumber("Attempting Side: ", side);
      //SmartDashboard.putNumber("Attempting Forward", forward);
      //SmartDashboard.putNumber("Attempting Turn", turnAngle);
 
      CommandScheduler.getInstance().schedule(new DriveFwdAndSideAndTurn(s_Swerve, false, forward, side, turnAngle));
    }
  }

}
