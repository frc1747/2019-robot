/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import com.tigerhuang.gambezi.Gambezi;
import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;
import lib.frc1747.controller.Xbox;
import lib.frc1747.subsytems.HBRSubsystem;

public class LiftElevator extends Command {
  
  public static Elevator elevator;
  public LiftElevator() {
    elevator = Elevator.getInstance();
    requires(elevator);
  }

  @Override
  protected void initialize() {
    elevator.setPower(0.7);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    GambeziDashboard.set_double("Elevator/Elevator Position", elevator.getDistance());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   return !OI.getInstance().getDriver().getButton(Xbox.X).get();
      // return !OI;
  }
  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.setPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
