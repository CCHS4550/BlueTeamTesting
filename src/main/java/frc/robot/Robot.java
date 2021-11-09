/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.parent.*;
import frc.util.*;
import frc.robot.sensors.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.util.ResourceBundle.Control;

// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotMap, ControlMap {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kCrossLine = "Cross Line";
  private static final String kCrossShoot = "Cross Line and Shoot";
  private static final String kCrossShootReload = "Cross Line, Shoot, and Reload";
  private static final String kTest = "Test Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  Compressor compressor = new Compressor();

  private static ReleasableButton yButton = new ReleasableButton();

  int alliance;
  double spdmlt = 1;


  int printCount;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    LemonLight.setLightChannel(0);
    
    resetAll();
    Logger.eraseLogs();
    Logger.logString("Robot Init");
    Solenoid.clearAllPCMStickyFaults(0);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Cross Line", kCrossLine);
    m_chooser.addOption("Cross Line and Shoot", kCrossShoot);
    m_chooser.addOption("Cross Line, Shoot, and Reload", kCrossShootReload);
    m_chooser.addOption("Test Auto", kTest);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putNumber("Shooter Spd", 1.0);

    printCount = 0;
  
    switch(DriverStation.getInstance().getAlliance()){
      case Blue:
        alliance = 1;
      break;

      case Red:
        alliance = 0; 
      break;
      
      case Invalid:
        alliance = -1;
      break;
    }
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
    if(printCount > 5)
    {
     // System.out.println("Angle: " + Chassis.getAngle());
    //  System.out.println("Position: " + Chassis.driveLeft.getPosition());
      // System.out.println(LemonLight.getYaw());
      // System.out.println(Loader.printSpin());
      // System.out.println("Turret Encoder: " + Turret.maxTurret.)
      // System.out.println("Dist to Target: " + LemonLight.distToTarget());
      // System.out.println("Shooter Velocity: " + Turret.getShooterVelocityFromDistance());
      // System.out.println("Shoter Current" + Turret.getShooterCurrent());

      // System.out.println("Chassis Pos: " + Chassis.getPos() / 12.0 + " ft.");
      // System.out.println("Chassis Angle: " + Chassis.getAngle());
      
      printCount = 0;
    }
    printCount++;

    if(RobotMap.ENABLE_COMPRESSOR)
    {
     compressor.start();
    }
    else
    {
     compressor.stop();
    }

    yButton.tick();

    // System.out.println(Loader.spinMax.getPosition());
    // System.out.println("Shoter Current" + Turret.getShooterCurrent());
    
      
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    resetAll();
    Logger.logString("Auto Init");

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
    System.out.println("Shooter Speed: " + SmartDashboard.getNumber("Shooter Spd", 0.0));

    Chassis.resetAll();
    Chassis.setTargetDistance(3, 1);
    Timer.delay(4);
    Chassis.turnAngle(90);
  
    
    
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Chassis.setDistance(8.0 * 12.0, 0.5);
    //     //Chassis.setAngle(5);
    //     break;
    //   case kCrossLine:
    //     Chassis.setTargetDistance(10.0 * 12.0, 0.3);
    //     break;
    //   case kCrossShoot:
    //     Chassis.setTargetDistance(4.0 * 12.0, 0.3);
    //     while(!(Chassis.getPos() < 3.9 * 12.0));
    //     Timer.delay(2.5);
    //     Turret.setShooterVelocity(Turret.getShooterVelocityFromDistance());
    //     Turret.lockOnForTime(0.75);
    //     Turret.setShooterVelocity(Turret.getShooterVelocityFromDistance());
    //     Timer.delay(1.5);
    //     Loader.reset();
    //     while(Loader.spinMax.getPosition() < 0.90)
    //     {
    //       Loader.setLoaderSpd(RobotMap.LOADER_FWD_SPEED);
    //       Loader.setSpinSpd(RobotMap.SPINDEXER_AUTO_SPEED);
    //     }
    //     Loader.setLoaderSpd(0.0);
    //     Loader.setSpinSpd(0.0);
    //     Turret.setShooterVelocity(0.0);
    //     break;
    //   case kCrossShootReload:
    //     Intake.setExtended(true);
    //     Intake.setSpd(0.55);
    //     Loader.setLoaderSpd(RobotMap.LOADER_REV_SPEED);
    //     Loader.setSpinSpd(RobotMap.SPINDEXER_SPEED);

    //     Chassis.setTargetDistance(10.0 * 12.0, 0.3); //Moves to pickup a single power cell from the trench run
    //     while(!(Chassis.getPos() < 9.75 * 12.0));
    //     Intake.setExtended(false);
    //     Intake.setSpd(0.0);

    //     Turret.setShooterVelocity(Turret.getShooterVelocityFromDistance());
    //     Turret.lockOnForTime(0.75);
    //     Turret.setShooterVelocity(Turret.getShooterVelocityFromDistance());
    //     while(Loader.spinMax.getPosition() < 0.90)
    //     {
    //       Loader.setLoaderSpd(RobotMap.LOADER_FWD_SPEED);
    //       Loader.setSpinSpd(RobotMap.SPINDEXER_AUTO_SPEED);
    //     }
    //     Turret.setShooterVelocity(0.0);

    //     Intake.setExtended(true);
    //     Intake.setSpd(0.55);
    //     Loader.setLoaderSpd(RobotMap.LOADER_REV_SPEED);
    //     Loader.setSpinSpd(RobotMap.SPINDEXER_SPEED);

    //     Chassis.setTargetDistance(5.0 * 12.0, 0.3); //Moves to pickup the remaining four power cells from the trench run
    //     while(!(Chassis.getPos() > 5.25 * 12.0));
        
    //     Intake.setExtended(false);
    //     Intake.setSpd(0.0);
    //     Loader.setLoaderSpd(0.0);
    //     Loader.setSpinSpd(0.0);
        
    //   case kDefaultAuto:
    //     break;
    //   case kTest:
    //     Chassis.setTargetDistance(5.0 * 1, 0.2);
    //     break;
    //   default:
        
    //     break;
    //}
    

    System.out.println("Auto Finished!");

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // Turret.setShooterVelocity(SmartDashboard.getNumber("Shooter Spd", 0.0));
  }

  @Override
  public void teleopInit() {
    resetAll();
    Logger.logString("Teleop Init");

    OI.initJoysticks();

    System.out.println("Shooter Speed: " + SmartDashboard.getNumber("Shooter Spd", 0.0));

    // Chassis.setFastMode(false);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  
    // //Basic Driving
    Chassis.driveAxis(OI.normalize(OI.axis(0, PilotMap.Y_AXIS), -RobotMap.DRIVE_SPD, RobotMap.DRIVE_SPD, 0.05), -OI.normalize(OI.axis(0, PilotMap.X_AXIS), -RobotMap.DRIVE_SPD, RobotMap.DRIVE_SPD, 0.05));
    //Chassis.driveAxis(OI.normalize(OI.axis(2, ControlMap.L_JOYSTICK_VERTICAL), -RobotMap.DRIVE_SPD, RobotMap.DRIVE_SPD, 0.05), -OI.normalize(OI.axis(2, ControlMap.L_JOYSTICK_HORIZONTAL), -RobotMap.DRIVE_SPD, RobotMap.DRIVE_SPD, 0.05));
    Chassis.drive();
    // //Shifting Gearbox Control
    Chassis.setFastMode(OI.button(0, PilotMap.TRIGGER));
   

    //Extends Intake, if extended, spin
    yButton.updateButton(OI.button(2, ControlMap.Y_BUTTON), 25);
    Intake.setExtended(yButton.getStatus());

    if(OI.button(2, ControlMap.A_BUTTON))
    {
      if(yButton.getStatus())
      {
        Intake.setSpd(RobotMap.INTAKE_SPD);
      }
      Loader.setSpinSpd(RobotMap.SPINDEXER_SPEED);
    }
    else
    {
      Intake.setSpd(0);
    }

    if(OI.axis(2, ControlMap.RT) > 0.05)
    {
      // Loader.setLoaderSpd(RobotMap.LOADER_FWD_SPEED);
      //Loader.setSpinSpd(RobotMap.SPINDEXER_SPEED);
    } 
    else 
    {
      // Loader.setLoaderSpd(RobotMap.LOADER_REV_SPEED);
      // Loader.setSpinSpd(0.0);
    }

  
    if(OI.normalize(OI.axis(2, ControlMap.LT), -1, 1) > 0.5)
    {
      Turret.lockOn();
      Turret.setShooterVelocity(Turret.getShooterVelocityFromDistance());
      // Turret.setShooterRaw(1.0);
      // Turret.setShooter(OI.normalize(OI.axis(2, ControlMap.LT), -1.0, 1.0, 0.1));
      // Turret.setShooter(Turret.getShooterSpeedFromDistance());
      // Turret.setShooter(1.0);
      // Turret.setShooterVelocity(SmartDashboard.getNumber("Shooter Spd", 0.0));
      // Loader.setSpinSpd(RobotMap.SPINDEXER_SPEED);
    }
    else
    {
      Turret.setSpin(OI.normalize(OI.axis(2, ControlMap.R_JOYSTICK_HORIZONTAL), -1.0, 1.0, 0.1));
      // Turret.setShooterRaw(0.0);
      if(OI.button(2, ControlMap.B_BUTTON))
      {
        Turret.setShooterVelocity(1.0);
      }
      else
      {
        Turret.setShooterVelocity(0.0);
      }
      if(!(OI.axis(2, ControlMap.RT) > 0.05) && !(OI.button(2, ControlMap.A_BUTTON)))
      {
        Loader.setSpinSpd(0.0);
      }
    }

    // if(OI.button(2, ControlMap.B_BUTTON))
    //   Turret.setShooterVelocity(Turret.getShooterVelocityFromDistance());
    

    if(OI.button(2, ControlMap.BACK_BUTTON))
    {
     Loader.setSpinSpd(-0.5);
      Turret.setShooterRaw(-0.15);
      Intake.setSpd(-0.4);
      // Climber.setRightSpd(-0.5);
    }

    

    if(OI.button(2, ControlMap.X_BUTTON))
    {
      Climber.setExtended(true);
    }
    else
    {
      Climber.setExtended(false);
    }

    if(OI.button(2, ControlMap.LB_BUTTON))
    {
      Climber.setLeftSpd(RobotMap.CLIMBER_SPD);
    }
    else
    {
      Climber.setLeftSpd(0.0);
    }
    if(OI.button(2, ControlMap.RB_BUTTON))
    {
      Climber.setRightSpd(RobotMap.CLIMBER_SPD);
    }
    else
    {
      Climber.setRightSpd(0.0);
    }

    // if(OI.normalize(OI.axis(2, ControlMap.LT), -1.0, 1.0) > 0.15)
    // {
    //   Climber.setLeftSpd(OI.axis(2, ControlMap.LT));
    // }
    // if(OI.normalize(OI.axis(2, ControlMap.RT), -1.0, 1.0) > 0.15)
    // {
    //   Climber.setRightSpd(OI.axis(2, ControlMap.RT));
    // }
  }

  // @Override
  // public void simulationInit() {
  //   Logger.logString("Simulation Init");
  // }
  

  @Override
  public void disabledInit() {
    Chassis.disableAll();
    LemonLight.setLight(false);

    Logger.logString("END");
    Logger.outputToFile("/home/lvuser/logFile-" + (int)(Math.random() * 100000000) + ".txt");
    Logger.outputToConsole();
    Logger.eraseLogs();

    Turret.maxTurret.setIdleMode(IdleMode.kCoast);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    LemonLight.setLight(true);
    //compressor.start();

  }

  private void resetAll()
  {
    Chassis.resetAll();
    Chassis.setPID(0.5, 0, 0, 0);
    Chassis.setFastMode(false);
    Intake.setExtended(false);
    Climber.setExtended(false);
    Turret.maxTurret.setPID(0.05, 0.0, 0.0, 0.0);
    Turret.maxShooter2.setPID(0.5, 0.0, 0.0, 0.0);
    Turret.reset();
    // Loader.reset();
    LemonLight.setLight(true);

    if(RobotMap.ENABLE_COMPRESSOR)
    {
     compressor.start();
    }
    else
    {
     compressor.stop();
    }

    System.out.println("RESET ALL");
  }
}