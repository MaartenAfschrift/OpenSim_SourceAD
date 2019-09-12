#ifndef OPENSIM_HUNT_CROSSLEY_FORCE_SMOOTH_H_
#define OPENSIM_HUNT_CROSSLEY_FORCE_SMOOTH_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  HuntCrossleyForce.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
// INCLUDE
#include "Force.h"
#include "OpenSim/Common/Set.h"

using namespace SimTK;
namespace OpenSim {

//==============================================================================
//                      HUNT CROSSLEY FORCE SMOOTH
//==============================================================================
/** This force subclass implements a Hunt-Crossley contact model. It uses Hertz
contact theory to model the interactions between a set of ContactSpheres and 
ContactHalfSpaces.
//
//@author Peter Eastman **/
class OSIMSIMULATION_API HuntCrossleyForce_smooth
	: public Force {
	OpenSim_DECLARE_CONCRETE_OBJECT(HuntCrossleyForce_smooth, Force);
public:

//==============================================================================
// PROPERTIES
//==============================================================================
//Real					RadiusSphere;
//Vec3					LocSphere;
//Body	BodySphere;

//Parameters			   parameters;

//==============================================================================
// PROPERTIES
//==============================================================================
//OpenSim_DECLARE_PROPERTY(contact_parameters, 
//    HuntCrossleyForce_smooth::ParametersSet,
//    "Material properties.");

OpenSim_DECLARE_PROPERTY(loc_sphere,
	Vec3,
	"Location where sphere is attached in body frame.");
OpenSim_DECLARE_PROPERTY(radius_sphere,
	Real,
	"Radius of the contact sphere.");
//OpenSim_DECLARE_LIST_PROPERTY(geometry, std::string,
//	"Names of geometry objects affected by these parameters.");

OpenSim_DECLARE_PROPERTY(stiffness, Real,
	"");
OpenSim_DECLARE_PROPERTY(dissipation, Real,
	"");
OpenSim_DECLARE_PROPERTY(static_friction, Real,
	"");
OpenSim_DECLARE_PROPERTY(dynamic_friction, Real,
	"");
OpenSim_DECLARE_PROPERTY(viscous_friction, Real,
	"");
OpenSim_DECLARE_PROPERTY(transition_velocity, Real,
	"");
OpenSim_DECLARE_PROPERTY(normal, Vec3,
	"");
OpenSim_DECLARE_PROPERTY(offset, Real,
	"");

    //OpenSim_DECLARE_PROPERTY(transition_velocity, double,
    //    "Slip velocity (creep) at which peak static friction occurs.");

//==============================================================================
// SOCKETS
//==============================================================================
OpenSim_DECLARE_SOCKET(body_sphere,
	PhysicalFrame,
	"Body where sphere is attached.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
HuntCrossleyForce_smooth();
//#ifndef SWIG
    /** The force takes ownership of the passed-in params. */
    /*HuntCrossleyForce(ContactParameters* params);*/
	HuntCrossleyForce_smooth(const std::string& name,
		const std::string& frame1Name, Vec3 locSphere, osim_double_adouble radiusSphere,
		osim_double_adouble stiffness, osim_double_adouble dissipation, osim_double_adouble staticFriction,
		osim_double_adouble dynamicFriction, osim_double_adouble viscousFriction, 
		osim_double_adouble transitionVelocity, Vec3 normal, osim_double_adouble offset);
	
//#endif

	//-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    OpenSim::Array<std::string> getRecordLabels() const override ;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<Real> getRecordValues(const SimTK::State& state) const override ;

protected:

    /**
     * Create a SimTK::Force which implements this Force.
     */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;


private:
    // INITIALIZATION
    void constructProperties();

//==============================================================================
};  // END of class HuntCrossleyForce
//==============================================================================
//==============================================================================
#ifndef SWIG

#endif

} // end of namespace OpenSim

#endif // OPENSIM_HUNT_CROSSLEY_FORCE_H_


