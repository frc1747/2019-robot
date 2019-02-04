/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScoring;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchPanelIntake;
import frc.robot.subsystems.HatchPanelScoring;
import frc.robot.subsystems.Extension;

public class Stop extends Command {
  Drivetrain drive;
  CargoIntake cIntake;
  HatchPanelIntake hIntake;
  CargoScoring cScore;
  HatchPanelScoring hScore;
  Elevator elevator;
  Extension extend;

  public Stop() {
    drive = Drivetrain.getInstance();
    cIntake = CargoIntake.getInstance();
    hIntake = HatchPanelIntake.getInstance();
    cScore = CargoScoring.getInstance();
    hScore = HatchPanelScoring.getInstance();
    elevator = Elevator.getInstance();
    extend = Extension.getInstance();
    requires(drive);
    requires(cIntake);
    requires(hIntake);
    requires(cScore);
    requires(hScore);
    requires(elevator);
    requires(extend);
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
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
