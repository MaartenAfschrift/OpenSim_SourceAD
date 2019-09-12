/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testExampleMain.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Cassidy Kelly                                                   *
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

// Author: Cassidy Kelly

//==============================================================================
//==============================================================================

//#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
//#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
//#include <OpenSim/Simulation/Model/Force.h>
//#include <OpenSim/Common/LinearFunction.h>
#include <adolc.h>


using namespace OpenSim;
using namespace SimTK;
using namespace std;


int main(){

	const std::string stri="HC_force";
	const std::string sphereName = "body_sphere";

	// Create a new OpenSim model on earth.
	auto osimModel = Model();
	osimModel.setGravity(Vec3(0, -9.80665, 0));
	// Create the body and attach it to the model.
	osim_double_adouble massbody1 = 1.0;
	auto body1Inertia = Inertia(1,1,1);
	auto body1 = new OpenSim::Body("body1", massbody1, Vec3(0), body1Inertia);
	osimModel.addBody(body1);
	
	/// Joints
	//PlanarJoint* ground_pelvis = new PlanarJoint("ground_body1", osimModel.getGround(), Vec3(0), Vec3(0), *body1, Vec3(0), Vec3(0));
    PlanarJoint* ground_foot = new PlanarJoint("ground_body1", 
		osimModel.getGround(), Vec3(0), Vec3(0), *body1, Vec3(0), Vec3(0));
    osimModel.addJoint(ground_foot);
	const osim_double_adouble c = 0.5;
	const osim_double_adouble us = 0.8;
	const osim_double_adouble ud = 0.8;
	const osim_double_adouble uv = 0.1;
	const osim_double_adouble vt = 0.001;

	//osim_double_adouble eps = 1e-16;
	//osim_double_adouble bd = 1000;
	//osim_double_adouble bv = 0.5;
	//osim_double_adouble bn = 100;
	
	/*HuntCrossleyForce_smooth HC();*/
	Vec3 normal = Vec3(0, 1, 0);
	osim_double_adouble offset = 0;

	//HC_heel = new HuntCrossleyForce_smooth("sphere_heel", "foot", locSphere_heel, radiusSphere_heel,
	//	stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
	Vec3 locSphere = Vec3(-0.05, 0.1, 0.3);
	osim_double_adouble radiusSphere = 0.2;
	osim_double_adouble stiffness = 5000;
	auto HC = new HuntCrossleyForce_smooth(stri, sphereName, locSphere, radiusSphere,
		stiffness, c, us, ud, uv, vt, normal, offset);
	osimModel.addComponent(HC);
	HC->connectSocket_body_sphere(*body1);
	
	//std::cout << osimModel.getForceSubsystem().getNumForces() << std::endl;

	// Initialize the system and state.
	State* state = new State(osimModel.initSystem());

	Vector q(3);
	q.setToZero();
	state->setQ(q);
	Vector u(3);
	u.setToZero();
	state->setU(u);

	osimModel.realizeVelocity(*state);
	
	Array<std::string> Labels_forces = HC->getRecordLabels();
	Array<osim_double_adouble> Force_values = HC->getRecordValues(*state);
	std::cout << Labels_forces << std::endl;
	std::cout << Force_values << std::endl;

}
