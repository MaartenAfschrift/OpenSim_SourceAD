#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <recorder.hpp>

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

/// number of inputs/outputs of function F
constexpr int n_in = 3;
constexpr int n_out = 1;

constexpr int NX = 4; // states
constexpr int NU = 2; // controls
constexpr int NP = 1; // perturbation
constexpr int NR = 2; // residual forces

/* Function F, using templated type T
F(x,u,p) -> (Tau)
*/

template<typename T>
T value(const Recorder& e) {
	return e;
}

template<>
double value(const Recorder& e) {
	return e.getValue();
}

template<typename T>
int F_generic(const T** arg, T** res) {

	// setup the model

	Model* model;
	OpenSim::Body* lowerbody;
	OpenSim::Body* upperbody;
	OpenSim::PinJoint* ankle;
	OpenSim::PinJoint* hip;

	State* state;

	// Define the system.
	model = new OpenSim::Model();
	lowerbody = new OpenSim::Body("lowerbody", 26, Vec3(0, 0.55, 0), Inertia(1.4));
	upperbody = new OpenSim::Body("upperbody", 46, Vec3(0, 0.365, 0), Inertia(2.9));

	// Connect the bodies with pin joints. Assume each body is 1 m long.
	ankle = new PinJoint("ankle",
		// Parent body, location in parent, orientation in parent.
		model->getGround(), Vec3(0), Vec3(0),
		// Child body, location in child, orientation in child.
		*lowerbody, Vec3(0), Vec3(0));
	hip = new PinJoint("hip",
		*lowerbody, Vec3(0, 0.853, 0), Vec3(0), *upperbody, Vec3(0, 0, 0), Vec3(0));

	// Add components to the model.
	model->addBody(lowerbody);    model->addBody(upperbody);
	model->addJoint(ankle);  model->addJoint(hip);

	// Initialize the system and state.
	state = new State(model->initSystem());


	// Read inputs
	std::vector<T> x(arg[0], arg[0] + NX);
	std::vector<T> u(arg[1], arg[1] + NU);
	std::vector<T> p(arg[2], arg[2] + NP);

	T pert[NP];
	T ua[NU];

	Vector QsUs(NX);

	// Assign inputs to model variables
	for (int i = 0; i < NX; ++i) QsUs[i] = x[i];    // states
	for (int i = 0; i < NU; ++i) ua[i] = u[i];      // controls
	pert[0] = p[0];

	model->setStateVariableValues(*state, QsUs);
	model->realizeVelocity(*state);

	int ndof = NX / 2;

	// appliedMobilityForces
	Vector appliedMobilityForces(ndof);
	appliedMobilityForces.setToZero();

	// appliedBodyForces
	Vector_<SpatialVec> appliedBodyForces;
	int nbodies = model->getBodySet().getSize() + 1; // including ground
	appliedBodyForces.resize(nbodies);
	appliedBodyForces.setToZero();
	/// Gravity
	Vec3 gravity(0);
	gravity[1] = -9.81;
	/// Perturbation
	Vec3 pertVec_gravity = Vec3(pert[0] * gravity[1], gravity[1], 0);
	/// Add to model
	for (int i = 0; i < model->getBodySet().getSize(); ++i) {
		model->getMatterSubsystem().addInStationForce(*state, model->getBodySet().get(i).getMobilizedBodyIndex(), model->getBodySet().get(i).getMassCenter(), pertVec_gravity*model->getBodySet().get(i).getMass(), appliedBodyForces);
	}

	// knownUdot
	Vector knownUdot(ndof);
	knownUdot.setToZero();
	knownUdot[0] = ua[0];
	knownUdot[1] = ua[1];

	// residualMobilityForces
	Vector residualMobilityForces(ndof);
	residualMobilityForces.setToZero();
	model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state, appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);

	// CasADi may not always request all outputs
	// if res[i] is a null pointer, this means that output i is not required
	if (res[0]) for (int i = 0; i < NR; ++i) res[0][i] = value<T>(residualMobilityForces[i]);

	return 0;

}

int main() {

	Recorder x[NX];
	Recorder u[NU];
	Recorder p[NP];
	Recorder tau[NR];

	for (int i = 0; i < NX; ++i) x[i] <<= 0;
	for (int i = 0; i < NU; ++i) u[i] <<= 0;
	for (int i = 0; i < NP; ++i) p[i] <<= 0;


	const Recorder* Recorder_arg[n_in] = { x,u, p };
	Recorder* Recorder_res[n_out] = { tau };

	F_generic<Recorder>(Recorder_arg, Recorder_res);

	double res[NR];
	for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

	Recorder::stop_recording();

	return 0;
}
