/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.Auton.AutonTarget;
import frc.robot.commands.Auton.Stop;
import frc.robot.commands.CargoIntake.Intake;
import frc.robot.commands.CargoScoring.AutonOutake;
import frc.robot.commands.CargoScoring.Outtake;
import frc.robot.commands.Climber.ExtendClimber;
import frc.robot.commands.Climber.ManualLift;
import frc.robot.commands.Drivetrain.DriveProfiles;
import frc.robot.commands.Drivetrain.DriveToTarget2;
import frc.robot.commands.Elevator.ElevatorProfiles;
import frc.robot.commands.Elevator.HatchFinal;
import frc.robot.commands.Elevator.ZeroElevator;
import frc.robot.commands.Extension.Extend;
import frc.robot.commands.General.MotorTest;
import frc.robot.commands.HatchPanelIntake.Handoff;
import frc.robot.commands.HatchPanelScoring.ActuateBeak;
import frc.robot.commands.HatchPanelScoring.Beak;
import frc.robot.commands.HatchPanelScoring.HPScore;
// import frc.robot.commands.HatchPanelScoring.WeakAuton;
import frc.robot.subsystems.Elevator;
import lib.frc1747.controller.Xbox;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Xbox driver;
  Xbox operator;
  Xbox programmer;
  static OI oi;

  public OI() {
    driver = new Xbox(RobotMap.DRIVER_PORT);
    operator = new Xbox(RobotMap.OPERATOR_PORT);
    programmer = new Xbox(RobotMap.PROGRAMMER_PORT);
    driver();
    operator();
    // programmer();
  }
  public void operator(){
    //operator.getDPad(Xbox.DOWN).whenPressed(new Extend());
    operator.getButton(Xbox.Y).whenPressed(new ElevatorProfiles(3));
    operator.getButton(Xbox.A).whenPressed(new ElevatorProfiles(4));
    operator.getButton(Xbox.X).whenPressed(new ElevatorProfiles(2));
    operator.getButton(Xbox.B).whenPressed(new ElevatorProfiles(1));
    operator.getDPad(Xbox.UP).whenPressed(new Intake(0, 0.7, false));
    operator.getDPad(Xbox.UP).whenReleased(new Intake(0, 0, false));

    operator.getDPad(Xbox.LEFT).whenPressed(new ExtendClimber(false, true));
    operator.getDPad(Xbox.LEFT).whenReleased(new ExtendClimber(true, true));
    
    // operator.getLTButton().whenPressed(new GroundIntake2());

    operator.getDPad(Xbox.RIGHT).whenPressed(new ExtendClimber(true, false));
    operator.getDPad(Xbox.RIGHT).whenReleased(new ExtendClimber(true, true));


    // operator.getButton(Xbox.START).whenPressed(new ZeroElevator());
    // operator.getButton(Xbox.BACK).whenPressed(new ManualWrist());
    
    operator.getButton(Xbox.BACK).whenPressed(new Stop());
    operator.getButton(Xbox.START).whenPressed(new Stop());
  }
  public void driver() {

    // un-comment this line below when camera mount is fixed
    driver.getButton(Xbox.A).whenPressed(new HPScore());
    driver.getButton(Xbox.BACK).whenPressed(new Stop());
    driver.getButton(Xbox.START).whenPressed(new Stop());


    // driver.getButton(Xbox.A).whenPressed(new SetWristPower());

    // driver.getButton(Xbox.X).whenPressed(new LiftElevator());
        
    // driver.getButton(Xbox.X).whenPressed(new DriveProfiles("/home/lvuser/s-curve_fwd_nor.csv"));

    driver.getRTButton().whenPressed(new Intake(0.9, 0.7, true));
    driver.getRTButton().whenReleased(new Intake(0, 0, false));

    // driver.getRTButton().whenPressed(new Outtake(1));
    driver.getButton(Xbox.RB).whenPressed(new Outtake(-1));
    driver.getButton(Xbox.RB).whenReleased(new AutonOutake(500, 0.7));

    // driver.getButton(Xbox.Y).whenPressed(new DriveTargetPath());
    driver.getButton(Xbox.Y).whenPressed(new DriveToTarget2());

    // driver.getButton(Xbox.Y).whenPressed(new AutonTarget(3000));
    driver.getButton(Xbox.LB).whenPressed(new Beak());

    // driver.getDPad(Xbox.UP).whenPressed(new Handoff());
    // driver.getDPad(Xbox.RIGHT).whenPressed(new HatchFinal());
  }

  public void programmer(){

    // programmer.getButton(Xbox.A).whenPressed(new DriveProfiles("/home/lvuser/Joseph_Bell_test1_fwd_mir.csv"));
    // programmer.getButton(Xbox.B).whenPressed(new DriveProfiles("/home/lvuser/Joseph_Bell_test2_rev_mir.csv"));    programmer.getButton(Xbox.A).whenPressed(new DriveProfiles("/home/lvuser/Joseph_Bell_test1_fwd_nor.csv"));
    // programmer.getButton(Xbox.Y).whenPressed(new DriveProfiles("/home/lvuser/pickup_to_close_cargo_rev_mir.csv"));    programmer.getButton(Xbox.A).whenPressed(new DriveProfiles("/home/lvuser/Joseph_Bell_test1_fwd_nor.csv"));
    // programmer.getButton(Xbox.X).whenPressed(new DriveProfiles("/home/lvuser/Joseph_Bell_test4_fwd_nor.csv"));
   
  // programmer.getButton(Xbox.A).whenPressed(new DriveProfiles("/home/lvuser/platform_to_close_side_cargo_fwd_mir.csv"));
  // programmer.getButton(Xbox.B).whenPressed(new DriveProfiles("/home/lvuser/close_side_cargo_to_pickup_rev_mir.csv"));
  // programmer.getButton(Xbox.Y).whenPressed(new DriveProfiles("/home/lvuser/pickup_to_center_cargo_rev_mir.csv"));

  programmer.getLTButton().whenPressed(new DriveProfiles("/home/lvuser/Quincys_Test_3_rev_mir.csv"));
  programmer.getRTButton().whenPressed(new AutonTarget(1750));
  programmer.getButton(Xbox.X).whenPressed(new ManualLift(0.8));
  programmer.getButton(Xbox.X).whenReleased(new ManualLift(0));
  programmer.getButton(Xbox.B).whenPressed(new ManualLift(-0.4));
  programmer.getButton(Xbox.B).whenReleased(new ManualLift(0));
  programmer.getButton(Xbox.Y).whenPressed(new Extend());
  programmer.getButton(Xbox.A).whenPressed(new ActuateBeak());
  programmer.getLTButton().whenPressed(new MotorTest());

}
  public void makeClimb(){
    // programmer.getButton(Xbox.A).whenPressed(new ClimberMacro());
  }

  public static OI getInstance() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }

  public Xbox getDriver() {
    return driver;
  }

  public Xbox getOperator(){
    return operator;
  }

  public Xbox getProgrammer(){
    return programmer;
  }
}
