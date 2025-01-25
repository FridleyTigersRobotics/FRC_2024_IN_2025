// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>
#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <fmt/printf.h>
#include <frc/filter/SlewRateLimiter.h>

#include <Drivetrain.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>

#include <Climber.h>
#include <networktables/NetworkTable.h>
#include <LimelightHelpers.h>
#include <cameraserver/CameraServer.h>




bool DummyController::GetRightBumper(void)
{
  return 0;
}
bool DummyController::GetLeftBumper(void)
{
  return 0;
}
bool DummyController::GetAButton(void)
{
  return 0;
}
bool DummyController::GetBButton(void)
{
  return 0;
}
bool DummyController::GetXButton(void)
{
  return 0;
}
bool DummyController::GetYButton(void)
{
  return 0;
}
float DummyController::GetRightTriggerAxis(void)
{
  return 0;
}
bool DummyController::GetStartButtonPressed(void)
{
  return 0;
}
bool DummyController::GetBackButtonPressed(void)
{
  return 0;
}


void Robot::RobotInit() {
  m_Drivetrain.m_imu.Reset();
  frc::CameraServer::StartAutomaticCapture();
  // Autonomous Chooser
  m_autoChooser.SetDefaultOption( kAutoNameDefault,         kAutoNameDefault );
  m_autoChooser.AddOption       ( kAutoDrive,               kAutoDrive );
  m_autoChooser.AddOption       ( kShootCenter,             kShootCenter );
  m_autoChooser.AddOption       ( kShootCenterPickupCenter, kShootCenterPickupCenter );
  m_autoChooser.AddOption       ( kShootLeftPickupLeft,     kShootLeftPickupLeft );
  m_autoChooser.AddOption       ( kShootRightPickupRight,   kShootRightPickupRight );
  m_autoChooser.AddOption       ( kCenterShootRun,          kCenterShootRun );
  m_autoChooser.AddOption       ( kShootRightPickupRightTest, kShootRightPickupRightTest );
  m_autoChooser.AddOption       ( kShootLeftPickupLeftTest, kShootLeftPickupLeftTest );
  m_autoChooser.AddOption       ( kShootRunLeft,            kShootRunLeft );
  frc::SmartDashboard::PutNumber("AutoModeInt", 0 );
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);

  m_LimeRotatePid.SetSetpoint( m_limeAngleOffset );

  #if 0
        frc::SmartDashboard::PutNumber( "Move_x",                  double{0} );
        frc::SmartDashboard::PutNumber( "Move_ArmEndPosition",     double{0} );
        frc::SmartDashboard::PutNumber( "Move_m_startXArmPosition",double{0} );
        frc::SmartDashboard::PutNumber( "Move_Goal",               double{0} );
        frc::SmartDashboard::PutNumber( "Move_Out",                double{0} );
        frc::SmartDashboard::PutNumber( "Move_Angle",              double{0} );
  #endif
}


 void Robot::DisabledInit() {

 }



 void Robot::TeleopInit() {
  m_fieldRelative = true;
  m_Climber.initClimber();

  m_Drivetrain.Driveinit();

  //m_DriveTargetAngle = 0;

  m_AutoXdirPid.SetTolerance( kXyPosTolerance,  kXyVelTolerance );
  m_AutoXdirPid.Reset( 0.0_m );

#if 0
  frc::SmartDashboard::PutNumber("LimeVelocityMax", m_limeVelMax);
  frc::SmartDashboard::PutNumber("LimeAccelMax",    m_limeAccMax);
  frc::SmartDashboard::PutNumber("LimePValue",           m_limeP);
  frc::SmartDashboard::PutNumber("LimeMaxOut",            m_limeMaxOutput);
  frc::SmartDashboard::PutNumber("LimeOutput",           0);
  frc::SmartDashboard::PutNumber("LimeOutputUnclamped",           0);
  frc::SmartDashboard::PutNumber("RotPValue2",            m_RotP);
  frc::SmartDashboard::PutNumber("LimeMinOut",            m_limeMinOutput);
  frc::SmartDashboard::PutNumber("LimeMinThresh",            m_limeMinThresh);
  frc::SmartDashboard::PutNumber("limeAngleOffset",            m_limeAngleOffset);
#endif
 }



void Robot::RobotPeriodic()
{

}




  void Robot::TeleopPeriodic() 
  { 

  double DriveDeadband = 0.1;
  double DriveX = frc::ApplyDeadband( -m_driveController.GetLeftY(), DriveDeadband );
  double DriveY = frc::ApplyDeadband( -m_driveController.GetLeftX(), DriveDeadband );

  double driveRotSpeedUnClamped = -m_DriveRotatePid.Calculate( m_Drivetrain.GetAngle() );
  double driveRotSpeed          = std::clamp( driveRotSpeedUnClamped, -0.5, 0.5 );

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate( DriveX ) * Drivetrain::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate( DriveY ) * Drivetrain::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this.
    const auto rotationSpeed = driveRotSpeed * Drivetrain::kMaxAngularSpeed;

    m_Drivetrain.SetSpeeds( xSpeed, ySpeed, rotationSpeed );

 double targetWrappedAngle  = 0;
  bool   angleChanged        = false;

  if(m_driveController.GetYButtonPressed())
  {
    targetWrappedAngle = 0;
    angleChanged = true;
  }

  if (m_driveController.GetBButtonPressed())
  {
    targetWrappedAngle = 90;
    angleChanged = true;
  }

  if (m_driveController.GetAButtonPressed())
  {
    targetWrappedAngle = 180;
    angleChanged = true;
  }

  if (m_driveController.GetXButtonPressed())
  {
    targetWrappedAngle = -90;
    angleChanged = true;
  }



    double y     = m_driveController.GetRightY();
  double x     = m_driveController.GetRightX();
  double mag   = sqrt( x * x + y * y );
  double angle = ((atan2( x , y ) * 180 / std::numbers::pi) +180) * -1;
  
  frc::SmartDashboard::PutNumber( "controler_mag",   mag );
  frc::SmartDashboard::PutNumber( "controler_angle", angle );

  if( mag > 0.9 )
  {
    targetWrappedAngle = angle;
    angleChanged       = true;
  }

  if ( angleChanged )
  {
    double unwrappedRobotAngle = m_Drivetrain.GetAngle();
    double wrappedRobotAngle   = m_Drivetrain.GetYaw();
    double angleDelta = targetWrappedAngle - wrappedRobotAngle;
    if ( angleDelta > 180 )
    {
      angleDelta -= 360;
    }
    if ( angleDelta <= -180 )
    {
      angleDelta += 360;
    }

    m_DriveTargetAngle = unwrappedRobotAngle + angleDelta;
  frc::SmartDashboard::PutNumber( "HEADING_unwrappedRobotAngle",       unwrappedRobotAngle);
  frc::SmartDashboard::PutNumber( "HEADING_angleDelta",       angleDelta);

  }


  double TriggerRotateSpeedMult = 2.0;
  m_DriveTargetAngle += TriggerRotateSpeedMult * frc::ApplyDeadband( m_driveController.GetRightTriggerAxis(), 0.05 );
  m_DriveTargetAngle -= TriggerRotateSpeedMult * frc::ApplyDeadband( m_driveController.GetLeftTriggerAxis(),  0.05 );

  if(m_driveController.GetRightBumper())
  {
    m_Drivetrain.m_YawOffset -= 3;
  }
  else
  {  
    if(m_driveController.GetLeftBumper())
    {
    m_Drivetrain.m_YawOffset += 3;
    }
  }



  m_DriveRotatePid.SetSetpoint( m_DriveTargetAngle );
   
   


   
    // Update all subsystems
    m_Drivetrain.updateDrivetrain( GetPeriod(), m_fieldRelative );

    
  }



#ifndef RUNNING_FRC_TESTS
int main() {
 
  return frc::StartRobot<Robot>();

}
#endif
