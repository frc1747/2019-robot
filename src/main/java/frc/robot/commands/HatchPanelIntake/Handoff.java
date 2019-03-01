/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanelIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Auton.Delay;
import frc.robot.commands.CargoScoring.AutonOutake;
import frc.robot.commands.Elevator.ElevatorProfiles;
import frc.robot.commands.HatchPanelScoring.ActuateHPScoring;
import frc.robot.subsystems.HatchPanelIntake;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class Handoff extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Handoff() {
    addSequential(new MakeSequential(
      new ActuateHPScoring(250, true),
      // new Delay(000),
      new MakeParallel(
        new WristProfile(HatchPanelIntake.Positions.HANDOFF),
        new AutonHPIntake(750)
      ),
      new MakeParallel(
        new AutonOutake(500, 0.7),
        new ActuateHPScoring(500, false)
      )
      // new ElevatorProfiles(2),
      // new WristProfile(HatchPanelIntake.Positions.STOWED),
      // new ElevatorProfiles(1)
    ));
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
