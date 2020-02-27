/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Harvester;


public class IndexerCaptain extends CommandBase {
  Harvester harvestMarket;
  Indexer indexerCaptain;
  boolean isEmpty;
  boolean isFull;

  /**
   * Creates a new IndexerCaptain.
   */
  public IndexerCaptain(Indexer inputIndexer, Harvester inputHarvester) {
    // Use addRequirements() here to declare subsystem dependencies.
    indexerCaptain = inputIndexer;
    harvestMarket = inputHarvester;

    addRequirements(indexerCaptain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isEmpty = indexerCaptain.getP0();
    isFull = indexerCaptain.isIndexerFull();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isFull) {
      // Shift the PC's up one level 
     indexerCaptain.Movement(0.25); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (indexerCaptain.getEncoderValue() < 2730) {
      return false; 
    } else {
      return true;
    }
  }
}
