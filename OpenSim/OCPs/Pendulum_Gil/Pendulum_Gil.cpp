#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>
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

constexpr int ndof = 3;
constexpr int NX = 2*ndof;  // states
constexpr int NU = ndof;    // controls
constexpr int NP = 1;       // perturbation
constexpr int NR = ndof;    // residual forces

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
	/// Declare Simbody objects
	OpenSim::Model* model;
	OpenSim::Body* bodyInfo1;
	OpenSim::Body* bodyInfo2;
	OpenSim::Body* bodyInfo3;
	OpenSim::PinJoint* pendulum1;
	OpenSim::PinJoint* pendulum2;
	OpenSim::PinJoint* pendulum3;

	SimTK::State* state;

	// Define the system.
	model = new OpenSim::Model();
	bodyInfo1 = new OpenSim::Body("body1", 26.0, Vec3(0, 0.55, 0), Inertia(1.4));
	bodyInfo2 = new OpenSim::Body("body2", 46.0, Vec3(0, 0.365, 0), Inertia(2.9));
	bodyInfo3 = new OpenSim::Body("body3", 30.0, Vec3(0, 0.5, 0), Inertia(3.0));

	// Connect the bodies with pin joints. Assume each body is 1 m long.
	pendulum1 = new PinJoint("pendulum1",
		// Parent body, location in parent, orientation in parent.
		model->getGround(), Vec3(0), Vec3(0),
		// Child body, location in child, orientation in child.
		*bodyInfo1, Vec3(0), Vec3(0));
	pendulum2 = new PinJoint("pendulum2",
		*bodyInfo1, Vec3(0, 0.853, 0), Vec3(0), *bodyInfo2, Vec3(0), Vec3(0));
	pendulum3 = new PinJoint("pendulum3",
		*bodyInfo2, Vec3(0, 0.6, 0), Vec3(0), *bodyInfo3, Vec3(0), Vec3(0));

	// Add components to the model.
	model->addBody(bodyInfo1);
	model->addBody(bodyInfo2);
	model->addBody(bodyInfo3);
	model->addJoint(pendulum1);
	model->addJoint(pendulum2);
	model->addJoint(pendulum3);

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
    for (int i = 0; i < NP; ++i) pert[i] = p[i];      // controls

	model->setStateVariableValues(*state, QsUs);
	model->realizeVelocity(*state);

	// appliedMobilityForces
	Vector appliedMobilityForces(ndof);
	appliedMobilityForces.setToZero();

	// appliedBodyForces
	Vector_<SpatialVec> appliedBodyForces;
	int nbodies = model->getBodySet().getSize() + 1; // including ground
	appliedBodyForces.resize(nbodies);
	appliedBodyForces.setToZero();
	// Weight and perturbation
    Vec3 gravity(0);
    gravity[1] = -9.81;
    gravity[0] = pert[0]*gravity[1];
    for (int i = 0; i < model->getBodySet().getSize(); ++i) {
        model->getMatterSubsystem().addInStationForce(*state,
            model->getBodySet().get(i).getMobilizedBodyIndex(),
            model->getBodySet().get(i).getMassCenter(),
            model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
    }
	// knownUdot
    Vector knownUdot(ndof);
    knownUdot.setToZero();
    for (int i = 0; i < ndof; ++i) knownUdot[i] = ua[i];
    // Residual forces
    Vector residualMobilityForces(ndof);
    residualMobilityForces.setToZero();
    model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
        appliedMobilityForces, appliedBodyForces, knownUdot,
        residualMobilityForces);

	// CasADi may not always request all outputs
	// if res[i] is a null pointer, this means that output i is not required
	if (res[0]) {
		for (int i = 0; i < NR; ++i) {
			res[0][i] = value<T>(residualMobilityForces[i]);
		}
	}
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


	const Recorder* Recorder_arg[n_in] = { x, u, p };
	Recorder* Recorder_res[n_out] = { tau };

	F_generic<Recorder>(Recorder_arg, Recorder_res);

	double res[NR];
	for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

	Recorder::stop_recording();

	return 0;
}

