// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
// import frc.robot.triggers.*;
import frc.robot.utilities.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot key utilities (DO THIS FIRST)
  private final FileLog log = new FileLog("A1");

  // Define robot subsystems  
  private final DriveTrain driveTrain = new DriveTrain(log);

  // Define controllers
  private final Joystick xboxController = new Joystick(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  private boolean rumbling = false;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    driveTrain.setDefaultCommand(new DriveWithJoystick(leftJoystick, rightJoystick, driveTrain, log));
  }

    /**
   * Define Shuffleboard mappings.
   */
  private void configureShuffleboard() {

    // display sticky faults
    RobotPreferences.showStickyFaultsOnShuffleboard();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log).ignoringDisable(true));

    // DriveTrain subsystem
    // SmartDashboard.putData("DriveForward", new DriveSetPercentOutput(0.4, 0.4, driveTrain, log));

    // DriveTrain calibration
    // SmartDashboard.putData("Drive Cal Slow", new DriveCalibrate(0.3, 35, 0.01, CalibrateMode.kStraight, driveTrain, log));

    // Testing for drivetrain autos and trajectories
    SmartDashboard.putData("Zero Gyro", new DriveZeroGyro(driveTrain, log));
    SmartDashboard.putData("Zero Odometry", new DriveResetPose(0, 0, 0, driveTrain, log));
    SmartDashboard.putData("Calibrate Drive Motors", new DriveCalibration(0.5, 12, 0.05, driveTrain, log));
    SmartDashboard.putData("Calibrate Turn Motors", new DriveTurnCalibration(1.0, 10, 0.2, driveTrain, log));
    SmartDashboard.putData("Drive Wheels 0 deg", new DriveSetState(0, 0, driveTrain, log, true));
    SmartDashboard.putData("Drive Wheels +85 deg", new DriveSetState(0, 85, driveTrain, log, true));
    SmartDashboard.putData("Drive Wheels +95 deg", new DriveSetState(0, 95, driveTrain, log, true));
    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, driveTrain, log, true ));

    // Testing for autos and trajectories
    // TODO Implement a trajectory follower.  See Team364's exampleAuto.java command.
    // SmartDashboard.putData("Drive Trajectory Relative", new DriveFollowTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryType.test.value], false, PIDType.kTalon, driveTrain, log));
    // SmartDashboard.putData("Drive Trajectory Curve Relative", new DriveFollowTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryType.testCurve.value], false, PIDType.kTalon, driveTrain, log));
    // SmartDashboard.putData("Drive Trajectory Absolute", new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log));  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
  }

  /**
   * Configures XBox buttons and controls
   */
  private void configureXboxButtons(){
    JoystickButton[] xb = new JoystickButton[11];
    //check povtrigger and axis trigger number bindings
    // Trigger xbPOVUp = new POVTrigger(xboxController, 0);
    // Trigger xbPOVRight = new POVTrigger(xboxController, 90);
    //Trigger xbPOVDown = new POVTrigger(xboxController, 180);
    // Trigger xbPOVLeft = new POVTrigger(xboxController, 270);
    
    // Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
    // Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);
    
    // right trigger shoots ball
    // xbRT.whenActive(new ShootSequence(uptake, feeder, shooter, log),false);

    // left trigger aim turret
    // xbLT.whenActive(new TurretTurnAngleTwice(TargetType.kVisionOnScreen, false, turret, pivisionhub, log));
    // xbLT.whenInactive(new TurretStop(turret, log));

    for (int i = 1; i < xb.length; i++) {
      xb[i] = new JoystickButton(xboxController, i);
    }
    
    //a - short shot distance
    // xb[1].whenHeld(new ShootSetup(false, 3100, pivisionhub, shooter, log));         
    
    //b - medium shot distance
    // xb[2].whenHeld(new ShootSetup(false, 3400, pivisionhub, shooter, log));        
 
    //y - long shot distance
    // xb[4].whenHeld(new ShootSetup(false, 4100, pivisionhub, shooter, log));        
    
    //x - shot speed using vision
    // xb[3].whileTrue(new ShootSetup(true, 3100, pivisionhub, shooter, log));        
    
    // LB = 5, RB = 6
    // xb[5].onTrue(new TurretSetPercentOutput(-0.1, turret, log));
    // xb[5].whenReleased(new TurretStop(turret, log));
    // xb[6].onTrue(new TurretSetPercentOutput(+0.1, turret, log));
    // xb[6].whenReleased(new TurretStop(turret, log));

    // back = 7, start = 8 
    // xb[7].whenHeld(new ShootSetup(false, 500, pivisionhub, shooter, log));   // micro shot for use in the pit
    // xb[8].whenHeld(new ClimberSetExtended(false,climber, log)); 

    // pov is the d-pad (up, down, left, right)
    // xbPOVUp.whenActive(new TurretTurnAngle(TargetType.kAbsolute, 0, 2, turret, pivisionhub, log));
    // xbPOVRight.whenActive(new TurretTurnAngle(TargetType.kAbsolute, 45, 2, turret, pivisionhub, log));
    // xbPOVLeft.whenActive(new TurretTurnAngle(TargetType.kAbsolute, -45, 2, turret, pivisionhub, log));
    //xbPOVDown.whenActive(new StopAllMotors(feeder, shooter, intakeFront, uptake, log));
  }

  /**
   * Define drivers joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // TODO Add a button binding to zero the Pose angle (after adding that feature to DriveResetPose).
    // If the robot angle drifts (or is turned on with the wrong facing), then this button can be used to 
    // reset the robot facing for field-oriented control.  Turn the robot so that it is facing away
    // from the driver, then press this button.

    // left joystick left button
    //left[1].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));
    // resets current angle to 0, keeps current X and Y
    left[1].onTrue(new DriveResetPose(0,driveTrain,log));
   
    // left joystick right button
    //left[2].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));

    // right joystick left button
    // right[1].onTrue(new IntakeExtendAndTurnOnMotors(intakeFront, uptake, log)); 

    // right joystick right button
    // right[2].onTrue(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));  
  }

  /** 
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP then DOWN, from LEFT to RIGHT
    // coP[1].onTrue(new IntakeToColorSensor(intakeFront, uptake, log)); 
    // coP[2].onTrue(new ShooterSetVelocity(InputMode.kSpeedRPM, ShooterConstants.shooterDefaultRPM, shooter, log)); 

    // coP[3].onTrue(new UptakeFeedBall(uptake, feeder, log)); 
    // coP[4].onTrue(new UptakeEjectBall(uptake, log)); 

    // coP[5].whenHeld(new ClimbSetPercentOutput(0.4, climb, log)); // manually raise climb arms, slowly
    // coP[6].whenHeld(new ClimbSetPercentOutput(-0.4, climb, log)); // manually lower climb arms, slowly
    
    // top row RED SWITCH
    // coP[8].onTrue(new StopAllMotors(feeder, shooter, intakeFront, uptake, log));

    // middle row UP then DOWN, from LEFT to RIGHT
    // coP[9].onTrue(new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPctTransfer, intakeFront, log)); // forward intake and transfer
    // coP[10].onTrue(new IntakeSetPercentOutput(-IntakeConstants.onPct, -IntakeConstants.onPctTransfer, intakeFront, log)); // reverse intake and transfer

    // coP[11].onTrue(new UptakeSetPercentOutput(-UptakeConstants.onPct, 0, uptake, log)); // reverse uptake
    // coP[12].onTrue(new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log)); // forward uptake

    // coP[13].onTrue(new FeederSetPercentOutput(-FeederConstants.onPct, feeder, log)); // reverse feeder
    // coP[14].onTrue(new FeederSetPercentOutput(FeederConstants.onPct, feeder, log)); // forward feeder

    // middle row UP OR DOWN, fourth button
    // coP[7].onTrue(new IntakePistonToggle(intakeFront, uptake, log)); 

    // bottom row UP then DOWN, from LEFT to RIGHT
    // coP[15].onTrue(new ClimberSetExtended(true,climber, log)); // climb extend
    // coP[16].onTrue(new ClimberSetExtended(false,climber, log)); // climb retract
  }


  /**
   * Sets the rumble on the XBox controller
   * @param percentRumble The normalized value (0 to 1) to set the rumble to
   */
	public void setXBoxRumble(double percentRumble) {
		xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.setRumble(RumbleType.kRightRumble, percentRumble);

    if (percentRumble == 0) rumbling = false;
    else rumbling = true;
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    Command m_autoCommand = null;
    return m_autoCommand;
  }


  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }

  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    driveTrain.setDriveModeCoast(true);     // When pushing a disabled robot by hand, it is a lot easier to push in Coast mode!!!!
    driveTrain.stopMotors();                // SAFETY:  Turn off any closed loop control that may be running, so the robot does not move when re-enabled.
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }  //    TODO May want to flash this to the driver with some obvious signal!
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");

    driveTrain.setDriveModeCoast(false);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");

    driveTrain.setDriveModeCoast(false);
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {

  }
}
