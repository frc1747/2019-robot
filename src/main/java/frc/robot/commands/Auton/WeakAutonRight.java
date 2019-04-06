/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.Elevator.ElevatorProfiles;
import frc.robot.commands.HatchPanelScoring.ActuateHPScoring;
import frc.robot.commands.HatchPanelScoring.HPScore;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class WeakAutonRight extends CommandGroup {
  /**
   * Add your docs here.
   */
  public WeakAutonRight() {
    addSequential(new MakeSequential(
      new  DriveProfiles("/home/lvuser/platform_to_rocket_far_fwd_mir.csv"),
      new AutonTarget(1500),
      //new  ElevatorProfiles(1),
      new  HPScore(),
      //new  ElevatorProfiles(1),
      new  DriveProfiles("/home/lvuser/test2_rev_mir.csv"),
      // new MakeParallel(
        // new  ActuateHPScoring(4000, true),
        // new  DriveProfiles("/home/lvuser/center_to_pickup_fwd_mir.csv"),
        new MakeParallel(
          new AutonTarget(1750),
          new ActuateHPScoring(751, true)
        ),
        new ActuateHPScoring(150, false),
        new DriveProfiles("/home/lvuser/test_rev_mir.csv"),
        // new DriveProfiles("/home/lvuser/mid_to_rocket_close_fwd_mir.csv"),
        new AutonTarget(1750),
        //new  ElevatorProfiles(1),
        new  HPScore()
        //new  ElevatorProfiles(1)
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
