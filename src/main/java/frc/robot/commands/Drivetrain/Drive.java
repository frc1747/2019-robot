/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import lib.frc1747.controller.Logitech;

public class Drive extends Command {
  Drivetrain drivetrain;
  Logitech driver;
  public Drive() {
    driver = OI.getInstance().getDriver();
    drivetrain = Drivetrain.getInstance();
    requires(drivetrain);
    

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
   drivetrain.setLeftPower(-driver.getAxis(Logitech.LEFT_VERTICAL) + driver.getAxis(Logitech.RIGHT_HORIZONTAL));
   drivetrain.setRightPower(-driver.getAxis(Logitech.LEFT_VERTICAL) - driver.getAxis(Logitech.RIGHT_HORIZONTAL));
   SmartDashboard.putNumber("Left Power", -driver.getAxis(Logitech.LEFT_VERTICAL) + driver.getAxis(Logitech.RIGHT_HORIZONTAL));
   SmartDashboard.putNumber("Right Power", -driver.getAxis(Logitech.LEFT_VERTICAL) - driver.getAxis(Logitech.RIGHT_HORIZONTAL));
  //  if(drivetrain.getLeftSpeed() < 5){
  //     drivetrain.setLeftPower(0.3);
  //     drivetrain.setRightPower(0.3);
  //  }
  //   else{
  //     drivetrain.setLeftPower(0);
  //     drivetrain.setRightPower(0);
  //  }
 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if((drivetrain.getLeftSpeed() + drivetrain.getRightSpeed()) / 2 >= 5) {
    //   return true;
    // }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivetrain.setLeftPower(0);
    drivetrain.setRightPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}