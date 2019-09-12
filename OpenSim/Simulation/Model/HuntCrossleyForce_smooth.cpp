/* -------------------------------------------------------------------------- *
 *                      OpenSim:  HuntCrossleyForce.cpp                       *
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

#include "HuntCrossleyForce_smooth.h"
//#include "ContactGeometry.h"
#include <OpenSim/Simulation/Model/Model.h>

#include "simbody/internal/HuntCrossleyForce_smooth.h"
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
//#include <adolc.h>

using namespace SimTK;
namespace OpenSim {

	//==============================================================================
	//                       HUNT CROSSLEY FORCE SMOOTH
	//==============================================================================
	// Uses default (compiler-generated) destructor, copy constructor, copy 
	// assignment operator.

	// Default constructor.
	HuntCrossleyForce_smooth::HuntCrossleyForce_smooth()
	{
		constructProperties();
	}

	// Take over ownership of supplied object.
	HuntCrossleyForce_smooth::HuntCrossleyForce_smooth(const std::string& name,
		const std::string& frame1Name, Vec3 locSphere, osim_double_adouble radiusSphere,
		osim_double_adouble stiffness, osim_double_adouble dissipation, osim_double_adouble staticFriction,
		osim_double_adouble dynamicFriction, osim_double_adouble viscousFriction,
		osim_double_adouble transitionVelocity, Vec3 normal, osim_double_adouble offset)
	{
		this->updSocket("body_sphere").setConnecteeName(frame1Name);
			
		constructProperties();
		set_loc_sphere(locSphere);
		set_radius_sphere(radiusSphere);
		set_stiffness(stiffness);
		set_dissipation(dissipation);
		set_static_friction(staticFriction);
		set_dynamic_friction(dynamicFriction);
		set_viscous_friction(viscousFriction);
		set_transition_velocity(transitionVelocity);
		set_normal(normal);
		set_offset(offset);
	}


	void HuntCrossleyForce_smooth::extendAddToSystem(SimTK::MultibodySystem& system) const
	{
		Super::extendAddToSystem(system);
		
		const Vec3& locSphere = get_loc_sphere();
		const osim_double_adouble& radiusSphere = get_radius_sphere();

		osim_double_adouble stiffness = get_stiffness();
		osim_double_adouble dissipation = get_dissipation();
		osim_double_adouble staticFriction = get_static_friction();
		osim_double_adouble dynamicFriction = get_dynamic_friction();
		osim_double_adouble viscousFriction = get_viscous_friction();
		osim_double_adouble transitionVelocity = get_transition_velocity();
		Vec3 normal = get_normal();
		osim_double_adouble offset = get_offset();

		SimTK::HuntCrossleyForce_smooth force(_model->updForceSubsystem());
		
		const PhysicalFrame& frameSphere = getConnectee<PhysicalFrame>("body_sphere");
		force.setStiffness(stiffness);
		force.setDissipation(dissipation);
		force.setStaticFriction(staticFriction);
		force.setDynamicFriction(dynamicFriction);
		force.setViscousFriction(viscousFriction);
		force.setTransitionVelocity(transitionVelocity);
		force.setGroundPlane(normal, offset);

		force.setBodySphere(frameSphere.getMobilizedBody());
		force.setLocSphere(locSphere);
		force.setRadiusSphere(radiusSphere);
			
		HuntCrossleyForce_smooth* mutableThis = const_cast<HuntCrossleyForce_smooth *>(this);
		mutableThis->_index = force.getForceIndex();
	}

	void HuntCrossleyForce_smooth::constructProperties()
	{

		constructProperty_loc_sphere(Vec3(0));
		constructProperty_radius_sphere(osim_double_adouble(0));
		constructProperty_stiffness(osim_double_adouble(0));
		constructProperty_dissipation(osim_double_adouble(0));
		constructProperty_static_friction(osim_double_adouble(0));
		constructProperty_dynamic_friction(osim_double_adouble(0));
		constructProperty_viscous_friction(osim_double_adouble(0));
		constructProperty_transition_velocity(osim_double_adouble(0));
		constructProperty_normal(Vec3(0));
		constructProperty_offset(osim_double_adouble(0));

	}

	
	//=============================================================================
	// Reporting
	//=============================================================================
	/**
	 * Provide names of the quantities (column labels) of the force value(s) reported
	 *
	 */
	OpenSim::Array<std::string> HuntCrossleyForce_smooth::getRecordLabels() const
	{
		OpenSim::Array<std::string> labels("");

		labels.append(getName() + ".onPlane" + ".force.X");
		labels.append(getName() + ".onPlane" + ".force.Y");
		labels.append(getName() + ".onPlane" + ".force.Z");
		labels.append(getName() + ".onPlane" + ".torque.X");
		labels.append(getName() + ".onPlane" + ".torque.Y");
		labels.append(getName() + ".onPlane" + ".torque.Z");

		labels.append(getName() + ".onSphere" + ".force.X");
		labels.append(getName() + ".onSphere" + ".force.Y");
		labels.append(getName() + ".onSphere" + ".force.Z");
		labels.append(getName() + ".onSphere" + ".torque.X");
		labels.append(getName() + ".onSphere" + ".torque.Y");
		labels.append(getName() + ".onSphere" + ".torque.Z");
		//}
   /* }*/

		return labels;
	}
	/**
	 * Provide the value(s) to be reported that correspond to the labels
	 */
	OpenSim::Array<osim_double_adouble> HuntCrossleyForce_smooth::
		getRecordValues(const SimTK::State& state) const
	{
		OpenSim::Array<osim_double_adouble> values(1);
		
		const PhysicalFrame& bodySphere = getConnectee<PhysicalFrame>("body_sphere");
		
		MobilizedBodyIndex mbi = bodySphere.getMobilizedBody();
		const auto& forceSubsys = _model->getForceSubsystem();
				
		const SimTK::Force& abstractForce = forceSubsys.getForce(_index);
		const auto& simtkForce = (SimTK::HuntCrossleyForce_smooth &)(abstractForce);

		SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
		SimTK::Vector_<SimTK::Vec3> particleForces(0);
		SimTK::Vector mobilityForces(0);
		
		simtkForce.calcForceContribution(state, bodyForces, particleForces,
			mobilityForces);

	  //on Ground
		const auto& thisBodyForce = bodyForces(0);
		SimTK::Vec3 forces = thisBodyForce[1];
		SimTK::Vec3 torques = thisBodyForce[0];

		/*Vec3 aux=&forces[0];*/
		values.append(3, &forces[0]);
		values.append(3, &torques[0]);

		//on Plane
		const auto& thisBodyForce2 = bodyForces(mbi);
		SimTK::Vec3 forces2 = thisBodyForce2[1];
		SimTK::Vec3 torques2 = thisBodyForce2[0];

		values.append(3, &forces2[0]);
		values.append(3, &torques2[0]);

		/*}*/


		return values;
	}

	// end of namespace OpenSim
}