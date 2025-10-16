// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaePivot extends SubsystemBase {
  
  private SparkMax algaePivotMotor;
  private SparkMaxConfig algaePivotConfig;

  private DigitalInput algaeLimit;
  private boolean isPivotException;

  private double desiredSpeed;
  //switched to using external encoder, no NEO motor
  private Encoder algaePivotEncoder;
  //private RelativeEncoder algaePivotEncoder;


  public AlgaePivot() {
    //****** MAKE SURE TO PROGRAM THE SPARKMAX AS BRUSHED!!! ********/
    algaePivotMotor = new SparkMax(Constants.MotorControllers.ID_ALGAE_PIVOT, MotorType.kBrushed);
    algaePivotEncoder = new Encoder(Constants.AlgaePivot.DIO_ENC_A, Constants.AlgaePivot.DIO_ENC_B);
    //algaePivotEncoder = algaePivotMotor.getEncoder();

    algaePivotConfig = new SparkMaxConfig();
    algaePivotConfig.inverted(true);
    algaePivotConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    //Must do things like invert and set current limits BEFORE callling the motor.configure class below
    algaePivotMotor.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    try {
      algaeLimit = new DigitalInput(Constants.AlgaePivot.DIO_LIMIT);
    } catch (Exception e)
    {
      isPivotException = true;
      SmartDashboard.putBoolean("Algae Extend Limit switch threw an exception", true);
    }

    desiredSpeed = 0;

  }

  //METHODS START HERE
  public boolean isLimit()
  {
    if (isPivotException) {
      return true;
    } else{
      return algaeLimit.get();
    }
  }

  public boolean isFullyExtended(){
    return (getPivotEncoder() <= Constants.AlgaePivot.ENC_REVS_MAX);
  }

  public void stopAlgaePivot()
  {
    algaePivotMotor.set(0);
  }

  public void resetPivotEncoder()
  {
   // algaePivotEncoder.setPosition(0);
    algaePivotEncoder.reset();
  }

  public double getPivotEncoder()
  {
   // return algaePivotEncoder.getPosition(); NEO with SparkMax
   //for extenal Bourne encoder (512 counts per rev):
   return algaePivotEncoder.getRaw(); //gets actual count unscaled by the 1, 2 or 4x scale
  //return coralPivotEncoder.get(); //gets count adjusted for the 1, 2 or 4x scale factor
  }

  public double getPivotSpeed()
  {
    return algaePivotMotor.get();
  }

 // public boolean atRetractLimit(){
 //   if ((getPivotSpeed() >= 0 || desiredSpeed >= 0) && isLimit()){ //positive speed means retracting
 //     return true;
  //     } else {
  //      return false;
  //     }
  //   }

  public void setAlgaePivotSpeed(double speed){  
    desiredSpeed = speed;
    if (speed <= 0){ //positive speed means extending
      //Extending
      if (isFullyExtended()){
        //SmartDashboard.putBoolean("speed positive and extending, so stop", true);
        stopAlgaePivot();
      } else {
        //SmartDashboard.putBoolean("speed positive and not fully extended, so go", true);
        algaePivotMotor.set(speed);
      }
    } 
    else 
    {//Retracting
      if (isLimit()){
       // SmartDashboard.putBoolean("algae pivot limit hit and retracting, so stop", true);
        //Added line below - assuming we should stop at retract limit - 
        //TODO: remove line blow if needed to hold PID
        stopAlgaePivot();
       // SmartDashboard.putBoolean("AP Limit Hit, so resetting encoder", true);
        resetPivotEncoder();
      } else {
       // SmartDashboard.putBoolean("algae pivot limot not hot and retracting, so go", true);
        algaePivotMotor.set(speed);
      }
    }
  }
  
  @Override
  public void periodic() {
   // SmartDashboard.putNumber("Algae Pivot speed is: ", getPivotSpeed());
    SmartDashboard.putBoolean("Algae Pivot limit is hit", isLimit());
    SmartDashboard.putBoolean("Algae Pivot is fully extended", isFullyExtended());
    SmartDashboard.putNumber("Algae Pivot Encoder revolutions", getPivotEncoder());
    SmartDashboard.putNumber("algae pivot speed ", getPivotSpeed());
  }
}