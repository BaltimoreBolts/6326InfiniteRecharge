/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  public CANSparkMax IndexerDonaldMotor;
  DigitalInput limitSwitch1;
  DigitalInput limitSwitch2;
  DigitalInput limitSwitch3;
  DigitalInput limitSwitch4;
  DigitalInput limitSwitch5;

  /**
   * Creates a new Indexer.
   */
  public Indexer() {
    IndexerDonaldMotor = new CANSparkMax (IndexerConstants.INDEXER_MOTOR_DONALD, MotorType.kBrushless); 
    limitSwitch1 = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH1);
    limitSwitch2 = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH2);
    limitSwitch3 = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH3);
    limitSwitch4 = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH4);
    limitSwitch5 = new DigitalInput(IndexerConstants.INDEXER_LIMIT_SWITCH5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
