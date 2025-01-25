#pragma once

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <rev/SparkMax.h>
using namespace rev::spark;
#include <numbers>
#include <frc/AnalogEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <rev/sparkmax.h>


class OldAnalogEncoder : 
  frc::AnalogEncoder
{
    using frc::AnalogEncoder::AnalogEncoder;

  public:
    double GetDistance( void ) const
    {
      return Get();
    }

    void SetDistancePerRotation( double distPerRot )
    {
      
    }

    void SetPositionOffset( double posOffset )
    {
      
    }

};







class SwerveModule {
 public:
  SwerveModule(
    int driveMotorChannel, 
    int turningMotorChannel,
    int turningEncoderChannel,
    const double turningEncoderOffset,
    units::meters_per_second_t maxSpeed
  );
  frc::SwerveModuleState GetState() const;
  frc::SwerveModulePosition GetPosition() const;
  void SetDesiredState(const frc::SwerveModuleState& state);

 private:
  static constexpr double kWheelRadius = 0.0508;
  static constexpr int kEncoderResolution = 4096;


  units::meters_per_second_t m_maxSpeed{ 0.0 };


  static constexpr auto kModuleMaxAngularVelocity =
      std::numbers::pi * 100_rad_per_s;  // radians per second

  static constexpr auto kModuleMaxAngularAcceleration =
      std::numbers::pi * 200_rad_per_s / 1_s;  // radians per second^2

  SparkMax m_driveMotor;
  SparkMax m_turningMotor;

  int m_drivechannel;
  std::string m_encodername;


  SparkRelativeEncoder m_driveEncoder = m_driveMotor.GetEncoder();
  OldAnalogEncoder m_turningEncoder;
//Line fourty-nine??? That's CRAZY
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      1.0,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  double m_PositionConversionFactor;
};