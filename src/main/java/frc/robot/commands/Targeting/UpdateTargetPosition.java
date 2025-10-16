// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateTargetPosition extends InstantCommand {
  private Swerve s_Swerve;

  public UpdateTargetPosition(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotFieldPose = s_Swerve.getPose(); //MUST be updated before this command runs, probably through UpdateRobotPosition

    // double[] targetPoseBotArray = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    // double[] targetPoseArray = LimelightHelpers.getTargetPose_CameraSpace("limelight"); //TODO: robot space might not be field space, in which case would need to add pose
    // SmartDashboard.putNumber("Target Side Dist From Cam (in):", Units.metersToInches(targetPoseArray[0]));
    // SmartDashboard.putNumber("Target Side Dist From Cam (m):", targetPoseArray[0]);
    // SmartDashboard.putNumber("Target Forward Dist From Cam (in):", Units.metersToInches(targetPoseArray[2]));
    // SmartDashboard.putNumber("Target Forward Dist From Cam (m):", targetPoseArray[2]);
    // SmartDashboard.putNumber("Target Angle from Robot:", targetPoseBotArray[4]);

    int targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    SmartDashboard.putNumber("Target ID", targetId);

    double x2 = Constants.Targeting.ID_TO_POSE.get(targetId).getX();
    double y2 = Constants.Targeting.ID_TO_POSE.get(targetId).getY();
    double angle2 = Constants.Targeting.ID_TO_POSE.get(targetId).getRotation().getRadians();
/* 
    SmartDashboard.putNumber("Robot Field X (m):", x2);
    SmartDashboard.putNumber("x2: ", Units.metersToInches(x2));
    SmartDashboard.putNumber("Robot Field Y (m):", y2);
    SmartDashboard.putNumber("y2: ", Units.metersToInches(y2));
    SmartDashboard.putNumber("angle2", Units.radiansToDegrees(angle2));
    */
  }
}