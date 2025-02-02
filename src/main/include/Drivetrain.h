// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <studica/AHRS.h>
#include "SwerveModule.h"
#include "Constants.h"
#include "frc/geometry/Rotation2d.h"
#include <frc/SPI.h>

using namespace ConstantCrap;

/**
 * Represents a swerve drive style drivetrain.
 */

//32^2=a^2+b^s    512

class Drivetrain {
 public:
  double m_YawOffset = 0;
  //double m_DriveTargetAngle = 0;
  Drivetrain() {m_imu.ResetDisplacement(); }

  void updateDrivetrain( units::second_t period, bool fieldRelative );

  void SetSpeeds(
    units::meters_per_second_t  xSpeed,
    units::meters_per_second_t  ySpeed, 
    units::radians_per_second_t rot
    );

  void AddToSpeeds(
    units::meters_per_second_t  xSpeed,
    units::meters_per_second_t  ySpeed, 
    units::radians_per_second_t rot
    );

  void UpdateOdometry();
  void Driveinit();
  void ResetYaw();
  void UpdateSmartDashboardData();
  double GetYaw();
  double GetAngle();
  static constexpr units::meters_per_second_t kMaxSpeed =
      1.0_mps;  // 3 meters per second


  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      0.5 * std::numbers::pi};  // 1/2 rotation per second

 
   units::meters_per_second_t  m_xSpeed{ 0.0 };
   units::meters_per_second_t  m_ySpeed{ 0.0 };
   units::radians_per_second_t m_rot   { 0.0 };

   //-----------|Front|------------
  //  10----------------------12
  //  |----YAY CODE TEAM!------|
  //  |------------------------|
  //  |------------------------|
  //  |------Tom is smart------|
  //  |------Eli is smart------|
  //  |----Mina is smartish----|
  //  |----Kewsar is smart-----|
  //  |----Hail Bing Skrong----|
  //  |-Bing Skrong is strong--|
  //  16----------------------14
  //------------|back|-------------
  /// 32 in diagonal
  // 22.627417
  //11.3137085
  frc::Translation2d m_frontLeftLocation {+0.28575_m, +0.28575_m};
  frc::Translation2d m_frontRightLocation{+0.28575_m, -0.28575_m};
  frc::Translation2d m_backLeftLocation  {-0.28575_m, +0.28575_m};
  frc::Translation2d m_backRightLocation {-0.28575_m, -0.28575_m};

//1.230863 Drive motor #10
//0.909437 Drive motor #12
//0.255626 Drive motor #14
//4.980153 Drive motor #16


  SwerveModule m_backRight { kBackRightDriveID,  kBackRightSpinID,  kDriveEncoderBackRight,  1.230863 / (2*std::numbers::pi),       kMaxSpeed };
  SwerveModule m_frontRight{ kFrontRightDriveID, kFrontRightSpinID, kDriveEncoderFrontRight, 0.5 + 0.909437 / (2*std::numbers::pi), kMaxSpeed };
  SwerveModule m_backLeft  { kBackLeftDriveID,   kBackLeftSpinID,   kDriveEncoderBackLeft,   0.255626 / (2*std::numbers::pi),       kMaxSpeed };
  SwerveModule m_frontLeft { kFrontLeftDriveID,  kFrontLeftSpinID,  kDriveEncoderFrontLeft,  4.980153 / (2*std::numbers::pi),       kMaxSpeed };

  studica::AHRS m_imu { studica::AHRS::NavXComType::kI2C };

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};



    // X Postive = Backwards
    // Y Positive = Right
    // Rot Positive = Clockwise

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      frc::Rotation2d{units::degree_t {GetYaw()}},//m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};

};
