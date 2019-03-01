/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanelScoring;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.CargoScoring;
import frc.robot.subsystems.HatchPanelScoring;
import lib.frc1747.controller.Xbox;

public class Beak extends Command {
  HatchPanelScoring hatch;
  // CargoScoring cargo;
  public Beak() {
    // cargo = CargoScoring.getInstance();
    hatch = HatchPanelScoring.getInstance();
    requires(hatch);
    // requires(cargo);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // cargo.setPower(0.7);
    hatch.setExtended(true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !OI.getInstance().getDriver().getButton(Xbox.LB).get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // cargo.setPower(0);
    hatch.setExtended(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
