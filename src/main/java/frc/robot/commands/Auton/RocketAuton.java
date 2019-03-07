/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.CargoIntake.AutonIntake;
import frc.robot.commands.CargoScoring.AutonOutake;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.Elevator.ElevatorProfiles;
import frc.robot.commands.Extension.AutonExtend;
import frc.robot.commands.HatchPanelScoring.ActuateHPScoring;
import frc.robot.subsystems.Elevator;
import lib.frc1747.commands.MakeParallel;
import lib.frc1747.commands.MakeSequential;

public class RocketAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RocketAuton() {
    addSequential(new MakeSequential(
      new MakeParallel(
        new MakeSequential(
          new Delay(2000),
          new ElevatorProfiles(2)
        ),
        new DriveProfiles("/home/lvuser/platform_to_rocket_far_fwd_nor.csv")
      ),  
      new MakeParallel(
        // new AutonExtend(100),
        new ActuateHPScoring(200, true)
      ),
      new MakeParallel(
        new ElevatorProfiles(1),
        new DriveProfiles("/home/lvuser/rocket_far_to_center_fwd_nor.csv")
      ),
      new DriveProfiles("/home/lvuser/center_to_pickup_fwd_nor.csv"),
      new ActuateHPScoring(200, false),
      new DriveProfiles("/home/lvuser/pickup_to_mid_fwd_nor.csv"),
      new MakeParallel(
        new MakeSequential(
          new Delay(1000),
          new ElevatorProfiles(2)
        ),
        new DriveProfiles("/home/lvuser/mid_to_rocket_close_fwd_nor.csv")
      ),
      new MakeParallel(
        // new AutonExtend(100),
        new ActuateHPScoring(200, true)
      ),
      new MakeParallel(
        new DriveProfiles("/home/lvuser/rocket_close_to_mid_fwd_nor.csv"),
        new ElevatorProfiles(1)
    ),
    new MakeParallel(
      new DriveProfiles("/home/lvuser/mid_to_cargo_fwd_nor.csv")
      // new AutonIntake()
    ),
    new DriveProfiles("/home/lvuser/cargo_to_mid_far_fwd_nor.csv"),
    new MakeParallel(
      new MakeSequential(
        new Delay(500),
        new ElevatorProfiles(2)
      ),
      new DriveProfiles("/home/lvuser/mid_far_to_rocket_front_fwd_nor.csv"),
      new AutonOutake(200, -0.5)
    )
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
