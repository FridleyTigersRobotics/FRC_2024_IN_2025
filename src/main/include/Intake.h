#pragma once

#include <Phoenix5.h>
#include <rev/SparkMax.h>
#include <Constants.h>
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <Utilities.h>

#include <rev/SparkMax.h>
using namespace rev::spark;

class Intake{
 public:
    typedef enum intake_movement
    {
        Intake_Intaking,
        Intake_IntakingWithSensor,
        Intake_Outtaking,
        Intake_Stopped,
    } intake_movement_t;

    Intake();
    void initIntake();
    void ChangeIntakeState(intake_movement_t);
    void updateIntake ();
    bool IsRingDetected();
    void UpdateSmartDashboardData();

  private:
    SparkMax m_IntakeMotor { ConstantCrap::kIntakeMotorcanID, SparkLowLevel::MotorType::kBrushless };
    frc::AnalogInput m_RingDetector{GetAnalogChannelFromPin(0)};
    intake_movement_t m_intake_movement;

    static constexpr double kIntakeSpeed  = -0.8;
    static constexpr double kOuttakeSpeed = 1.0;

};
