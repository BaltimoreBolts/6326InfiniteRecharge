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
import com.revrobotics.AlternateEncoderType;
import java.lang.Math;

public class Indexer extends SubsystemBase {
  public CANSparkMax IndexerDonaldMotor;
  DigitalInput limitSwitch1;
  CANEncoder indexencoder;
  private CANEncoder alternateEncoder;
  private static final int kCPR = 8192;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private CANPIDController indexerPID;
  double kP = 2e-5; 
  double kI = 0; 
  double kD = 0; 
  double kFF = 0.000165;//0.000015;
  ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");
  NetworkTableEntry pTemp = indexerTab.add("Current pVal = ", 0).getEntry();
  NetworkTableEntry iTemp = indexerTab.add("Current iVal = ", 0).getEntry();
  NetworkTableEntry dTemp = indexerTab.add("Current dVal = ", 0).getEntry();
  NetworkTableEntry ffTemp = indexerTab.add("Current ffVal = ", 0).getEntry();
  NetworkTableEntry desiredRotationNT = indexerTab.add("Desired Rotation = ", 0).getEntry();
  NetworkTableEntry currentRotationNT = indexerTab.add("Current Rotation = ", 0).getEntry();


  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    IndexerDonaldMotor = new CANSparkMax (IndexerConstants.INDEXER_MOTOR_DONALD, MotorType.kBrushless); 
    limitSwitch1 = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH1);
    indexencoder = IndexerDonaldMotor.getAlternateEncoder();
    alternateEncoder = IndexerDonaldMotor.getAlternateEncoder(kAltEncType, kCPR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("indexencoder", alternateEncoder.getPosition());
  }
  public int degreeToCounts(double degrees, int CPR ){
    int Counts = 0;
    Counts = (int)Math.ceil(CPR * degrees/360.0);
    return Counts;

  }
  public void PIDTuner() {
    double p = 0;
    double i = 0;
    double d = 0;
    double ff = 0;
    double desiredRotation = 0;

    p = pTemp.getDouble(0);
    i = iTemp.getDouble(0);
    d = dTemp.getDouble(0);
    ff = ffTemp.getDouble(0);
    desiredRotation = desiredRotationNT.getDouble(0);   
    
    if((p != kP)) { indexerPID.setP(p); kP = p; }
    if((kI != i)) { indexerPID.setI(i); kI = i; }
    if((d != kD)) { indexerPID.setD(d); kD = d; }
    if((ff != kFF)) { indexerPID.setFF(ff); kFF = ff; }
    int desiredCounts = degreeToCounts(desiredRotation, kCPR);
    int currentCounts = (int)alternateEncoder.getPosition();
    indexerPID.setReference(desiredCounts + currentCounts, ControlType.kPosition);
    currentRotationNT.setDouble(alternateEncoder.getPosition());
  }  
}
