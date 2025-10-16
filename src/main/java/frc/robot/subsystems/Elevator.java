// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX leftElevatorMotor, rightElevatorMotor;
  private TalonFXConfigurator leftConfig, rightConfig;
  private TalonFXConfiguration leftTalonConfig, rightTalonConfig;

  private CurrentLimitsConfigs leftCurrentConfigs, rightCurrentConfigs;
  private MotorOutputConfigs leftOutputConfigs, rightOutputConfigs;

  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isTException, isBException;

  private MotionMagicVoltage m_request;

  public Elevator() {
    leftElevatorMotor = new TalonFX(Constants.MotorControllers.ID_ELEVATOR_LEFT_TALON, "usb");

    // configure motors
    leftTalonConfig = new TalonFXConfiguration();
    leftTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    leftTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  // set slot 0 gains
    var slotLConfigs = leftTalonConfig.Slot0;
    slotLConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slotLConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slotLConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slotLConfigs.kP = 2;//4.8; // A position error of 2.5 rotations results in 12 V output
    slotLConfigs.kI = 0; // no output for integrated error
    slotLConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionLMagicConfigs = leftTalonConfig.MotionMagic;
    motionLMagicConfigs.MotionMagicCruiseVelocity = 80;//80; // Target cruise velocity of 80 rps
    motionLMagicConfigs.MotionMagicAcceleration = 120;//160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionLMagicConfigs.MotionMagicJerk = 1200;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    leftElevatorMotor.getConfigurator().apply(leftTalonConfig);

    
  
    rightElevatorMotor = new TalonFX(Constants.MotorControllers.ID_ELEVATOR_RIGHT_TALON, "usb");

    rightTalonConfig = new TalonFXConfiguration();
    //rightTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    rightTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

     // set slot 0 gains
    var slotRConfigs = rightTalonConfig.Slot0;
    slotRConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slotRConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slotRConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slotRConfigs.kP = 2;//4.8; //START LOWER??// A position error of 2.5 rotations results in 12 V output
    slotRConfigs.kI = 0; // no output for integrated error
    slotRConfigs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionRMagicConfigs = rightTalonConfig.MotionMagic;
    motionRMagicConfigs.MotionMagicCruiseVelocity = 80;//80; // Target cruise velocity of 80 rps
    motionRMagicConfigs.MotionMagicAcceleration = 120;//160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionRMagicConfigs.MotionMagicJerk = 1200;//1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    rightElevatorMotor.getConfigurator().apply(rightTalonConfig);

    rightElevatorMotor.setControl(new Follower(Constants.MotorControllers.ID_ELEVATOR_LEFT_TALON, false));

    m_request = new MotionMagicVoltage(0);

    try {
      elevatorTopLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_TOP);
    } catch (Exception e) {
      isTException = true;
      SmartDashboard.putBoolean("exception thrown for elev bottom limit: ", isTException);
    }

    try {
      elevatorBottomLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_BOTTOM);
    } catch (Exception e) {
      isBException = true;
      SmartDashboard.putBoolean("exception thrown for elev bottom limit: ", isBException);
    }
  }

  //METHODS START HERE

  public void stopElevator() {
    leftElevatorMotor.set(0);
   // rightElevatorMotor.set(0);
  }

  public boolean isETopLimit() {
    if (isTException) {
      return true;
    } else {
      return elevatorTopLimit.get();
    }
  }
  public boolean isEBotLimit() {
    if (isBException) {
      return true;
    } else {
      return elevatorBottomLimit.get();
    }
  }

  // reset/zero encoders
  public void resetElevatorEncoders(){
    leftElevatorMotor.setPosition(0);
   rightElevatorMotor.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS (number of rotations)
  public double getElevLeftEncoder() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }
    public double getElevRightEncoder() {
    return rightElevatorMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorHeight(){
    return Constants.Elevator.ELEV_REV_TO_IN * ((getElevLeftEncoder() + getElevRightEncoder()) / 2);
  }

    public double getElevatorHeightMeters(){
      return Constants.Elevator.ELEV_REV_TO_METERS* ((getElevLeftEncoder() + getElevRightEncoder()) / 2);
    }

 // check if elevator is at "top" according to user definition
  public boolean isTop() {
    return (getElevatorHeight() >= Constants.Elevator.MAX_HEIGHT);
  }

  public void setElevSpeed(double speed) {
   
    if (speed > 0) {  
      if (isETopLimit() || isTop()) {
          // if elevator limit is tripped or elevator is near the top limit switch going up, stop 
          stopElevator();
       }  else {
          // elevator going up but top limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
         // rightElevatorMotor.set(speed); //remove vbecause right follows top
        }
      } 
    else {
      if (isEBotLimit()) {
        //elevator going down and is at the bottom,stop and zero encoder
        stopElevator();
        resetElevatorEncoders();
      } else {
      // elevator going down but not at the bottom, go at commanded speed
        leftElevatorMotor.set(speed);
       // rightElevatorMotor.set(speed);
      }
    }  
  } 

  public double getElevatorLeftSpeed() {
    return leftElevatorMotor.get();
  }  

  public double getElevatorRightSpeed() {
    return rightElevatorMotor.get();
  }  

  public void doMotionMagic(double desiredRevs){
    leftElevatorMotor.setControl(m_request.withPosition(desiredRevs));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator height: ", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator at top limit: ", isETopLimit());
    SmartDashboard.putBoolean("Elevator at bottom limit: ", isEBotLimit());
    SmartDashboard.putBoolean("Elevator at Max height: ", isTop());
    //SmartDashboard.putNumber("Elevator speed: ", (getElevatorLeftSpeed() + getElevatorRightSpeed())/2);
    SmartDashboard.putNumber("Elevator left enc revs = ", getElevLeftEncoder());
    SmartDashboard.putNumber("Elevator right enc revs = ", getElevRightEncoder());

  }
}
