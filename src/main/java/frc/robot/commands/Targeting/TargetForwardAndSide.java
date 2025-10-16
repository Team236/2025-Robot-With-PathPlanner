// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetForwardAndSide extends Command {  //Targeting with Limeight
// Basic targeting data
//tx =  Horizontal offset from crosshair to target in degrees
//ty = Vertical offset from crosshair to target in degrees
//ta = Target area (0% to 100% of image)
//tv = hasTarget, Do you have a valid target?
        // 3D Pose Data
        //.getRobotPose_FieldSpace();    // Robot's pose in field space
        //.getCameraPose_TargetSpace();   // Camera's pose relative to tag
        // .getRobotPose_TargetSpace();     // Robot's pose relative to tag
        // .getTargetPose_CameraSpace();   // Tag's pose relative to camera
        //.getTargetPose_RobotSpace();     // Tag's pose relative to robot
        //Below, X is the sideways distance from target, Y is down distance, Z is forward distance
        // 3D pose array contains [0] = X, [1] = Y, [2] = Z, [3] = roll,  [4] = pitch, (all in meters)
        //                        [5] = yaw in degrees

        private double standoffForward; // desired Forward distance in inches from bumper to tag; pass into command
        private double standoffSideways; // desired sideways distance in inches from bumper to tag; pass into command
        private double errorFwd, errorSide, poseFwd, poseSide, poseAngle;
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  //In this case, angular velocity will be set proportional to pose angle (LL-to-target horizontal offset angle,)
  //forward speed will be proportional to the forward distance between the robot bumper and the Apriltag,
  //and sideways speed will be proportional to side-to-side distance between the center of the LL lens and the center of the AprilTag
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kProtation = Constants.Targeting.KP_ROTATION;//kP value for rotation
    double kPtranslation = Constants.Targeting.KP_TRANSLATION;//kP value for forward (translation) motion
    double kPstrafe = Constants.Targeting.KP_STRAFE;  //kP value for the sideways 
    private double tv, pipeline;
    
    private Swerve s_Swerve;    
  
  /** Creates a new TargetAllParallel. */
  public TargetForwardAndSide(Swerve s_Swerve, double standoffForward, double standoffSideways) {
    this.s_Swerve = s_Swerve;
    this.standoffForward = standoffForward;
    this.standoffSideways = standoffSideways;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    s_Swerve.zeroHeading(); //added this to fix the targeting going the wrong way
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv == 1) { //tv =1 means Limelight sees a target

    //poseFwd is the third element [2] in the pose array, which is the forward distance from center of LL camera to the AprilTag
    poseFwd = LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]; 
    //Standsoff is from bumper to Target. Must add forward dist from bumper to LLcamera (since using TargetPose-CameraSpace)
    double finalForward = Units.inchesToMeters(standoffForward + Constants.Targeting.DIST_CAMERA_TO_BUMPER_FWD);
    errorFwd = poseFwd - finalForward; 
    double targetingForwardSpeed = errorFwd*kPtranslation;
    //SmartDashboard.putNumber("Forward distance from Robot frame to tag in inches: ", ((dz/0.0254)-Constants.Targeting.DIST_CAMERA_TO_BUMPER_FWD));
    double translationVal = targetingForwardSpeed;

    //poseSide is first element in the pose array - which is sideways distance from center of LL camera to the AprilTag in meters  
    poseSide = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0];
    double finalSideways = Units.inchesToMeters(standoffSideways);  //convert desired standoff from inches to meters
    errorSide = poseSide - finalSideways; //OR DO WE NEED ADD finalStandoff here instead of subtract it?
    double targetingSidewaysSpeed = errorSide*kPstrafe;
   // SmartDashboard.putNumber("Side to side distance - camera to target, in inches: ", dx/0.0254);
    targetingSidewaysSpeed *= -1.0;  //IS NEEDED
    double strafeVal = targetingSidewaysSpeed;

   /* Drive */
   s_Swerve.drive(
       new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
       0,
       true,  //true for robot centric
       true //true for open loop (?)
   );
    }

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    //INSERT CODE TO STOP HERE?
    s_Swerve.drive(
      new Translation2d(0, 0), 
      0 , 
      true,  //true for robot centric
      true //true for open loop (?)
  );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return ((Math.abs(errorFwd) < Units.inchesToMeters(0.3)) && (Math.abs(errorSide) < Units.inchesToMeters(0.3)) &&  (poseAngle < 1));
    return false;
  }

}
