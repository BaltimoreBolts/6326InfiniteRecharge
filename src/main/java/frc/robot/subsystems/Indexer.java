/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants;
import frc.robot.Constants.GenConstants;
import com.revrobotics.AlternateEncoderType;
import java.lang.Math;

public class Indexer extends SubsystemBase {
  private CANSparkMax IndexerDonaldMotor;
  private DigitalInput OpticalSensor;
  private CANEncoder alternateEncoder;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;

  ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
  NetworkTableEntry desiredRotationNT = indexerTab.add("Desired Rotation = ", 0).getEntry();
  NetworkTableEntry currentRotationNT = indexerTab.add("Current Rotation = ", 0).getEntry();
  NetworkTableEntry desiredSpeedNT = indexerTab.add("Desired Speed = ", 0).getEntry();


  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    IndexerDonaldMotor = new CANSparkMax (IndexerConstants.INDEXER_MOTOR_DONALD, MotorType.kBrushless); 
    OpticalSensor = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH1);
    alternateEncoder = IndexerDonaldMotor.getAlternateEncoder(kAltEncType, 
                        Constants.GenConstants.REV_ENCODER_CPR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    UpdateDashboard();
  }
  public int degreeToCounts(double degrees, int CPR ){
    int Counts = 0;
    Counts = (int)Math.ceil(CPR * degrees/360.0);
    return Counts;

  }
  /*Publish values we want to look at to dashboard */
  public void UpdateDashboard() {
    currentRotationNT.setDouble(alternateEncoder.getPosition());
    desiredSpeedNT.getDouble(0);
    desiredRotationNT.getDouble(0);
  } 
  //Move the indexer motor at a certain speed
  public void Movement (double speed){
    IndexerDonaldMotor.set(speed);
  } 
  public int getEncoderValue(){
    return (int)alternateEncoder.getPosition();
  }
  //Return value of first position optical sensor
  public boolean getP0(){
    return OpticalSensor.get();
  }

  //For testing, this will be disabled later
  public double getdesiredSpeed(){
    return desiredSpeedNT.getDouble(0);
  }
}
