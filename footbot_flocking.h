#ifndef FOOTBOT_FLOCKING_H
#define FOOTBOT_FLOCKING_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;
class CFootBotFlocking : public CCI_Controller {

public:
   struct SWheelTurningParams {
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   struct SFlockingInteractionParams {
      /* Target robot-robot distance in cm */
      Real TargetDistance;
      /* Gain of the Lennard-Jones potential */
      Real Gain;
      /* Exponent of the Lennard-Jones potential */
      Real Exponent;

      void Init(TConfigurationNode& t_node);
      Real GeneralizedLennardJones(Real f_distance);
   };

public:

   /* Class constructor. */
   CFootBotFlocking();

   /* Class destructor. */
   virtual ~CFootBotFlocking() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void ControlStep();
   virtual void Reset();
   virtual void Destroy() {}

protected:
   virtual CVector2 VectorToLight();
   virtual CVector2 FlockingVector();
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;

   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;
   /* The flocking interaction parameters. */
   SFlockingInteractionParams m_sFlockingParams;
};

#endif
