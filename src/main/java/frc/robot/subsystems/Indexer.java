/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import com.revrobotics.AlternateEncoderType;

public class Indexer extends SubsystemBase {
  public CANSparkMax IndexerDonaldMotor;
  DigitalInput limitSwitch1;
  CANEncoder indexencoder;
  private CANEncoder m_alternateEncoder;
  private static final int kCPR = 8192;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
 

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    IndexerDonaldMotor = new CANSparkMax (IndexerConstants.INDEXER_MOTOR_DONALD, MotorType.kBrushless); 
    limitSwitch1 = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH1);
    indexencoder = IndexerDonaldMotor.getAlternateEncoder();
    m_alternateEncoder = IndexerDonaldMotor.getAlternateEncoder(kAltEncType, kCPR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
