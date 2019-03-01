/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Joystick.ButtonType;
// import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auton.Autonomous;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScoring;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchPanelIntake;
import lib.frc1747.controller.Xbox;
import lib.frc1747.subsytems.HBRSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  static DigitalInput jumper;
  TalonSRX motor1 = new TalonSRX(1);
  Drivetrain drivetrain;
  Joystick joyCon = new Joystick(0);
  Elevator elevator;
  CargoIntake cargoIntake;
  CargoScoring cargoScoring;
  HatchPanelIntake HPIntake;
  Command auton;
  AutonChoice[] choices;
  AutonPosition[] positions;
  int pos;
  int choice;
  HatchPanelIntake hpintake;
  PowerDistributionPanel dist;
  boolean lastAState;
  boolean lastBState;  

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    dist = new PowerDistributionPanel();
    positions = AutonPosition.class.getEnumConstants();
    choices = AutonChoice.class.getEnumConstants();
    // AutonChoice.class.getEnumConstants();
    cargoIntake = CargoIntake.getInstance();
    drivetrain = Drivetrain.getInstance();
    m_oi = new OI();
    elevator = Elevator.getInstance();
    HPIntake = HatchPanelIntake.getInstance();
    cargoScoring = CargoScoring.getInstance();
    hpintake = HatchPanelIntake.getInstance();
    lastAState = false;
    lastBState = false;
    
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);



    GambeziDashboard.set_double("Drivetrain/Distance P", 0.64); // 0.64
    GambeziDashboard.set_double("Drivetrain/Distance D", 0);
    GambeziDashboard.set_double("Drivetrain/Distance kV", 0.07); // 0.07
    GambeziDashboard.set_double("Drivetrain/Distance kA", 0.01); // 0.01

    GambeziDashboard.set_double("Drivetrain/Angle P", 2); // 2
    GambeziDashboard.set_double("Drivetrain/Angle D", 0);
    GambeziDashboard.set_double("Drivetrain/Angle kV", 0.08); // 0.08
    GambeziDashboard.set_double("Drivetrain/Angle kA", 0.01); // 0.01

    GambeziDashboard.set_double("Drivetrain/teleop Distance P", -0.02);
    GambeziDashboard.set_double("Drivetrain/teleop Distance D", 0);
    GambeziDashboard.set_double("Drivetrain/teleop Distance kV", 1./RobotMap.S_V_MAX);
    GambeziDashboard.set_double("Drivetrain/teleop Distance kA", 0);

    GambeziDashboard.set_double("Drivetrain/teleop Angle P", 0.14);
    GambeziDashboard.set_double("Drivetrain/teleop Angle D", 0);
    GambeziDashboard.set_double("Drivetrain/teleop Angle kV", 1./(RobotMap.A_V_MAX * 3));
    GambeziDashboard.set_double("Drivetrain/teleop Angle kA", 0);


    GambeziDashboard.set_string("Auton", "Rocket");

    GambeziDashboard.set_double("Elevator/kV", 0.0125);
    GambeziDashboard.set_double("Elevator/kA", 0.0015);
    GambeziDashboard.set_double("Elevator/P", 0.18);
    GambeziDashboard.set_double("Elevator/I", 0);
    GambeziDashboard.set_double("Elevator/D", 0.012);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    GambeziDashboard.set_double("Drivetrain/Left Encoder", drivetrain.getLeftDistance());
    GambeziDashboard.set_double("Drivetrain/Right Encoder", drivetrain.getRightDistance());
    GambeziDashboard.set_double("Wrist/Position", HPIntake.getWristPosition());
    SmartDashboard.putNumber("Wrist Gravity Force", Math.cos(HPIntake.getWristPosition())*RobotMap.GRAVITY_CONSTANT);
    GambeziDashboard.set_double("Elevator/Encoder", elevator.getDistance());
    Scheduler.getInstance().run();
    if(OI.getInstance().getDriver().getButton(Xbox.A).get() == true && lastAState == false){
      pos++;
      lastAState = true;
    }else if(lastAState == true && !OI.getInstance().getDriver().getButton(Xbox.A).get()){
      lastAState = false;
    }
    if(OI.getInstance().getDriver().getButton(Xbox.B).get() == true && lastBState == false){
      choice++;
      lastBState = true;
    }else if(lastBState == true && !OI.getInstance().getDriver().getButton(Xbox.B).get()){
      lastBState = false;
    }
    GambeziDashboard.set_string("Auton Choice", choices[choice%2].toString());
    GambeziDashboard.set_string("Auton Position", positions[pos%3].toString());
    GambeziDashboard.set_boolean("Robot", jumper.get());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
    auton = new Autonomous(choices[choice%2],positions[pos%3]);
    // auton = new WeakAuton();
    auton.start();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    
  }

  @Override
  public void teleopInit() {
    //drivetrain.resetGyro();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Scheduler.getInstance().removeAll();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      drivetrain.getRightSide().resetEncoder();
      drivetrain.getLeftSide().resetEncoder();
    }
    elevator.setMode(Elevator.Follower.ELEVATOR, HBRSubsystem.Mode.PID);
    elevator.setSetpoint(Elevator.Follower.ELEVATOR, elevator.getDistance());
    elevator.tellSetpoint(elevator.getDistance());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putNumber("left feet", drivetrain.getLeftDistance());
    SmartDashboard.putNumber("right feet", drivetrain.getRightDistance());
    GambeziDashboard.set_double("left current", drivetrain.getLeftCurrent());
    GambeziDashboard.set_double("right current", drivetrain.getRightCurrent());
    GambeziDashboard.set_double("left speed", drivetrain.getLeftSpeed());
    GambeziDashboard.set_double("right speed", drivetrain.getRightSpeed());
    GambeziDashboard.set_double("robot angle", drivetrain.getAngle());
    SmartDashboard.putNumber("Elevator Distance", elevator.getDistance());
    SmartDashboard.putNumber("robot angle", drivetrain.getAngle());
    SmartDashboard.putNumber("angular velocity", drivetrain.getAngularVelocity());
    SmartDashboard.putNumber("Wrist Position", HPIntake.getWristPosition());
    SmartDashboard.putBoolean("Robot", jumper.get());

    Scheduler.getInstance().run();
    SmartDashboard.putNumber("Wrist Voltage", HPIntake.getWristVolt());
    SmartDashboard.putBoolean("Has Ball", cargoScoring.sensorActivated());
    GambeziDashboard.set_double("channel 4-7 current", dist.getCurrent(4)+dist.getCurrent(5)+dist.getCurrent(6)+dist.getCurrent(7) - 6.125);
    GambeziDashboard.set_double("channel 8-11 current",  dist.getCurrent(8) + dist.getCurrent(9)+dist.getCurrent(10)+dist.getCurrent(11) - 7);
    // SmartDashboard.putNumber("channel 4", dist.getCurrent(4));
    // SmartDashboard.putNumber("channel 5", dist.getCurrent(4));
    // SmartDashboard.putNumber("channel 6", dist.getCurrent(4));
    // SmartDashboard.putNumber("channel 7", dist.getCurrent(4));
    // SmartDashboard.putNumber("channel 8", dist.getCurrent(4));
    // SmartDashboard.putNumber("channel 9", dist.getCurrent(4));
    // SmartDashboard.putNumber("channel 10", dist.getCurrent(4));
    // SmartDashboard.putNumber("channel 11", dist.getCurrent(4));





    // hpintake.setRollerPower(-0.5);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public enum AutonPosition{
    LEFT, CENTER, RIGHT;
  }
  
  public enum AutonChoice{
    ROCKET, SHIP;
  }
  public static DigitalInput getJumper(){
    if(jumper == null){
     jumper = new DigitalInput(7);
    }
    return jumper;
  }
}
