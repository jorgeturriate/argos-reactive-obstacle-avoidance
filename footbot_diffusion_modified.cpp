/*
 * Controller: Improved Reactive Obstacle Avoidance
 *
 * This controller implements a behavior-based obstacle avoidance strategy
 * inspired by Braitenberg Vehicles (Braitenberg, 1984).
 *
 * Improvements over the default ARGoS diffusion controller:
 *
 * 1. Continuous speed modulation instead of discrete in-place rotations,
 *    resulting in higher average forward velocity.
 *
 * 2. Vector-based fusion of proximity sensors to estimate obstacle
 *    direction and intensity.
 *
 * 3. Symmetry-breaking steering bias when obstacles are detected in front,
 *    preventing deadlock situations caused by symmetric sensor activation.
 *
 * The controller computes a weighted obstacle vector from proximity
 * readings and derives braking and steering components to adjust
 * differential wheel velocities.
 *
 * Reference:
 * Braitenberg, V. (1984). Vehicles: Experiments in Synthetic Psychology.
 */

/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>


/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {

   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );

   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Initialize wheel speeds with nominal forward velocity */
   Real fLeftSpeed  = m_fWheelVelocity;
   Real fRightSpeed = m_fWheelVelocity;

   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   /* Compute obstacle direction using vector summation of all proximity sensors.
   * Each sensor contributes a vector whose magnitude is the detected intensity
   * and whose angle corresponds to sensor orientation.
   */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   
   CRadians cAngle = cAccumulator.Angle(); //Resultant Vector Angle (Obstacle Orientation)
   Real fValue= cAccumulator.Length(); // Resultant Vector Magnitude (Obstacle Intensity)

   /* Forward braking component:
   * Reduces wheel speeds proportionally to obstacle proximity
   * in the forward direction.
   */
   Real fBrake = Cos(cAngle)* fValue* 5.0f;
   
   /* Steering component:
   * Generates differential wheel speeds to turn away from obstacles.
   * Based on lateral component of the accumulated obstacle vector.
   */
   Real fSteer = - Sin(cAngle)* fValue * 20.0f;
   
   /* Symmetry-breaking steering bias:
   * When an obstacle is detected directly in front, steering forces
   * from left and right sensors may cancel out, causing the robot
   * to continue moving forward toward the obstacle.
   *
   * To prevent this deadlock, an additional steering bias is injected,
   * forcing the robot to choose a turning direction.
   */
   if ( Abs(cAngle)< ToRadians(CDegrees(40.0))){
      fSteer += (cAngle.GetValue()>0.0f) ? -fValue*120.0f : fValue*120.0f;
   }
   /* Applying the Braking Component that slows down both wheel speeds*/
   fLeftSpeed  -= fBrake; 
   fRightSpeed -= fBrake;

   /* Applying the Steering Component that turns away from obstacles*/
   fLeftSpeed  -= fSteer;
   fRightSpeed += fSteer;

   /* Prevent negative speeds by setting the minimum wheel speed as 0.5 */
   Real fMinSpeed = 0.5f;

   if(fLeftSpeed  < fMinSpeed) fLeftSpeed  = fMinSpeed;
   if(fRightSpeed < fMinSpeed) fRightSpeed = fMinSpeed;

   /* Apply final computed velocities to the differential drive actuator*/
   m_pcWheels->SetLinearVelocity(fLeftSpeed, fRightSpeed);

}

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
