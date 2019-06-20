/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CargoScoring;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.CargoScoring;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Extension;
import lib.frc1747.controller.Xbox;

public class Outtake extends Command {
  CargoScoring intake;
  Extension extend;
  double power;
  public Outtake(double power) {
    extend = Extension.getInstance();
    this.power = power;
    intake = CargoScoring.getInstance();
    requires(intake);
    requires(extend);

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(Elevator.getInstance().getSetPoint() == 34){
        extend.setExtended(false);
        intake.setPower(-0.75);
    }else{   
        extend.setExtended(true);
        intake.setPower(power);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (!OI.getInstance().getDriver().getButton(Xbox.RB).get() && !OI.getInstance().getOperator().getDPad(Xbox.UP).get());  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    extend.setExtended(false);
    intake.setPower(0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
