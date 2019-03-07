/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CargoIntake;

import com.tigerhuang.gambezi.Gambezi;
import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScoring;
import lib.frc1747.controller.Xbox;

public class Intake extends Command {
  CargoIntake intake;
  double inPower;
  double scPower;
  CargoScoring scoring;
  boolean extend;
  public Intake(double inPower, double scPower, boolean extend) {
    this.inPower = inPower;
    this.scPower = scPower;
    this.extend = extend;

    intake = CargoIntake.getInstance();
    requires(intake);
    scoring = CargoScoring.getInstance();
    requires(scoring);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    intake.setPower(inPower);
    scoring.setPower(scPower);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intake.setExtended(extend);
    GambeziDashboard.set_double("Intake Distance", intake.getCurrent());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (scoring.sensorActivated());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.setPower(0.0);
    scoring.setPower(0.0);
    intake.setExtended(false);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
