#pragma once

#include "rev/SparkMax.h"
#include <Constants.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/math.h>
#include <units/time.h>

class Shooter
{
public:

    Shooter();
    void initShooter();
    void updateShooter ();
    bool ReadyToShoot();
    void changeShooterState ( bool spinUpShooter );
    void UpdateSmartDashboardData();
    bool shooterReadyToShoot();

private:
    CANSparkMax          m_shooterMotor   { ConstantCrap::kShooterMotorID, CANSparkLowLevel::MotorType::kBrushless };
    SparkPIDController   m_shooterPid     { m_shooterMotor.GetPIDController() };
    SparkRelativeEncoder m_shooterEncoder { m_shooterMotor.GetEncoder(SparkRelativeEncoder::Type::kHallSensor, 42) };


    // default PID coefficients
    double kP = 0.0002, kI = 0.0, kD = 0, kIz = 0, kFF = 0.0002, kMaxOutput = 1.0, kMinOutput = -0.1;

    // default smart motion coefficients
    double kMaxVel = 1750, kMinVel = 0, kMaxAcc = 0.00001, kAllErr = 0;


    // TODO : Determine the acceleration.
    frc::SlewRateLimiter<units::scalar> m_AccelerationLimiter{1 / 2_s};

    bool   m_spinUpShooter { false };
    bool   m_shooterSpeedReadyToShoot { false };

    // TODO : Determine all thse values.
    double const m_maxShooterSpeed{ 3000 };
    double const m_maxAccelOutput { 1.0 };
};