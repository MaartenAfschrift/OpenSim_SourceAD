#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <adolc.h>


using namespace SimTK;
using namespace OpenSim;

int main() {

	Model* model = new Model();

	// Create two links, each with a mass of 1 kg, center of mass at the body's
	// origin, and moments and products of inertia of zero.
	OpenSim::Body* humerus = new OpenSim::Body("humerus", 1, Vec3(0), Inertia(0));
	OpenSim::Body* radius = new OpenSim::Body("radius", 1, Vec3(0), Inertia(0));

	// Connect the bodies with pin joints. Assume each body is 1 m long.
	PinJoint* shoulder = new PinJoint("shoulder",
		// Parent body, location in parent, orientation in parent.
		model->getGround(), Vec3(0), Vec3(0),
		// Child body, location in child, orientation in child.
		*humerus, Vec3(0, 1, 0), Vec3(0));
	PinJoint* elbow = new PinJoint("elbow",
		*humerus, Vec3(0), Vec3(0), *radius, Vec3(0, 1, 0), Vec3(0));

	// Add a path actuator
	PathActuator* onefakemuscle = new PathActuator();
	onefakemuscle->addNewPathPoint("origin", *humerus, Vec3(0, 0.8, 0));
	onefakemuscle->addNewPathPoint("insertion", *radius, Vec3(0, 0.7, 0));

	// Add components to the model.
	model->addBody(humerus);    model->addBody(radius);
	model->addJoint(shoulder);  model->addJoint(elbow);
	model->addForce(onefakemuscle);

	//Coordinate coordshoulder = shoulder->getCoordinate();

	// Configure the model.
	//State state = model->initSystem();

    State* state = new State(model->initSystem());

	// Create a moment arm solver
	MomentArmSolver maSolver(*model);

	// Independent variables
	Vector qad(2);
	Vector uad(2);
	adouble* pert = new adouble[1];

	// Set independent variable values
	double* xp = new double[5];
	xp[0] = 1.2;
	xp[1] = 1.6;
	xp[2] = 1.3;
	xp[3] = 1.9;
	xp[4] = 1;

	//trace_on(1, 1);

	qad[0] <<= xp[0];
	uad[0] <<= xp[1];
	qad[1] <<= xp[2];
	uad[1] <<= xp[3];
	pert[0] <<= xp[4];

	state->setQ(qad);
	state->setU(uad);

	//std::cout << state->getQ() << std::endl;
	//std::cout << state->getU() << std::endl;
	//State tempstate = *(const State*) state;
	//adouble temp = 3.0;
	//tempstate.updQ() = temp * state->getQ();
	//std::cout << state->getQ() << std::endl;
	//std::cout << state->getU() << std::endl;

	//trace_off();

	//model->realizeVelocity(*state);

	//// appliedMobilityForces
	//Vector appliedMobilityForces(2);
	//appliedMobilityForces.setToZero();

	//// appliedBodyForces
	//Vector_<SpatialVec> appliedBodyForces;
	//appliedBodyForces.resize(3);
	//appliedBodyForces.setToZero();
	///// Gravity
	//Vec3 gravity(0);
	//gravity[1] = -9.81;
	///// Perturbation
	//Vec3 pertVec = Vec3(pert[0] * gravity[1], 0, 0);
	///// Weight
	//Vec3 weighthumerus = humerus->getMass()*gravity;
	//Vec3 weightradius = radius->getMass()*gravity;
	///// Weight + Perturbation
	//Vec3 weight_pert_humerus = weighthumerus + pertVec;
	//Vec3 weight_pert_radius = weightradius + pertVec;
	///// Add to model
	////model->getMatterSubsystem().addInStationForce(*state, humerus->getMobilizedBodyIndex(), humerus->getMassCenter(), weight_pert_humerus, appliedBodyForces);
	////model->getMatterSubsystem().addInStationForce(*state, radius->getMobilizedBodyIndex(), radius->getMassCenter(), weight_pert_radius, appliedBodyForces);
	//model->getMatterSubsystem().addInStationForce(state, humerus->getMobilizedBodyIndex(), humerus->getMassCenter(), weight_pert_humerus, appliedBodyForces);
	//model->getMatterSubsystem().addInStationForce(state, radius->getMobilizedBodyIndex(), radius->getMassCenter(), weight_pert_radius, appliedBodyForces);

	//// knownUdot
	//Vector knownUdot(2);
	//knownUdot.setToZero();

	//// residualMobilityForces
	//Vector residualMobilityForces(2);
	//residualMobilityForces.setToZero();
	////model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state, appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);
	//model->getMatterSubsystem().calcResidualForceIgnoringConstraints(state, appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);

	//std::cout << "resMobForces " << residualMobilityForces << std::endl;

	//// muscle length
	////adouble musclelength = onefakemuscle->getLength(*state);
	////adouble musclelengtheningspeed = onefakemuscle->getLengtheningSpeed(*state);
	//adouble musclelength = onefakemuscle->getLength(state);
	//adouble musclelengtheningspeed = onefakemuscle->getLengtheningSpeed(state);
	////std::cout << "muscleLength " << musclelength << std::endl;
	////std::cout << "muscleLengthSpeed " << musclelengtheningspeed << std::endl;

	std::cout << "first call solver => shoulder " << std::endl;
	std::cout << state->getQ() << std::endl;
	std::cout << state->getU() << std::endl;
	adouble momentarmshoulder = maSolver.solve(*state, shoulder->getCoordinate(), onefakemuscle->getGeometryPath());
	std::cout << "momentarmshoulder " << momentarmshoulder.value() << std::endl;
	std::cout << " " << std::endl;

	std::cout << "second call solver => shoulder " << std::endl;
	std::cout << state->getQ() << std::endl;
	std::cout << state->getU() << std::endl;
	adouble momentarmshoulder2 = maSolver.solve(*state, shoulder->getCoordinate(), onefakemuscle->getGeometryPath());
	std::cout << "momentarmshoulder " << momentarmshoulder2.value() << std::endl;
	std::cout << " " << std::endl;

	std::cout << "first call solver => shoulder" << std::endl;
	std::cout << state->getQ() << std::endl;
	std::cout << state->getU() << std::endl;
	adouble momentarmelbow = maSolver.solve(*state, elbow->getCoordinate(), onefakemuscle->getGeometryPath());
	std::cout << "momentarmelbow " << momentarmelbow.value() << std::endl;
	std::cout << " " << std::endl;

    std::cout << "second call solver => shoulder" << std::endl;
    std::cout << state->getQ() << std::endl;
    std::cout << state->getU() << std::endl;
    adouble momentarmelbow2 = maSolver.solve(*state, elbow->getCoordinate(), onefakemuscle->getGeometryPath());
    std::cout << "momentarmelbow " << momentarmelbow2.value() << std::endl;
    std::cout << " " << std::endl;

    std::cout << "first call computeMomentArm => shoulder " << std::endl;
    adouble momentarmshoulder_cma1 = onefakemuscle->getGeometryPath().computeMomentArm(*state, shoulder->getCoordinate());       
    std::cout << "momentarmshoulder " << momentarmshoulder_cma1.value() << std::endl;
    std::cout << " " << std::endl;

    std::cout << "first call computeMomentArm => elbow " << std::endl;
    adouble momentarmelbow_cma1 = onefakemuscle->getGeometryPath().computeMomentArm(*state, elbow->getCoordinate());
    std::cout << "momentarmshoulder " << momentarmelbow_cma1.value() << std::endl;
    std::cout << " " << std::endl;

	//adouble momentarmshoulder = maSolver.solve(*state, shoulder->getCoordinate(), onefakemuscle->getGeometryPath());
	//adouble momentarmelbow = maSolver.solve(*state, elbow->getCoordinate(), onefakemuscle->getGeometryPath());
	//adouble momentarmshoulder3 = onefakemuscle->computeMomentArm(*state, shoulder->updCoordinate());
	//adouble momentarmelbow3 = onefakemuscle->computeMomentArm(*state, elbow->updCoordinate());
	//std::cout << momentarmshoulder3.value() << std::endl;
	//std::cout << momentarmelbow3.value() << std::endl;

	//// test multiplyBySystemJacobianTranspose: ok
	//Vector _generalizedForces;
	//model->getMultibodySystem().getMatterSubsystem().multiplyBySystemJacobianTranspose(*state, appliedBodyForces, _generalizedForces);
	//std::cout << _generalizedForces << std::endl;

	// test addInEquivalentForces: not sure that makes sense
	//const GeometryPath& geometrypathonefakemuscle = onefakemuscle->getGeometryPath();
	//Vector pathDependentMobilityForces(state->getNU(), 0.0);
	//std::cout << appliedBodyForces << std::endl;
	//std::cout << state->getQ() << std::endl;
	//std::cout << state->getU() << std::endl;
	//geometrypathonefakemuscle.addInEquivalentForces(*state, 1.0, appliedBodyForces, pathDependentMobilityForces);
	//std::cout << appliedBodyForces << std::endl;

	//// Moment arms from scratch
	///// constructor
	//// 1st call
	//std::cout << state->getQ() << std::endl;
	//std::cout << state->getU() << std::endl;

	//std::cout << "First call from scratch" << std::endl;
	//Vector_<SimTK::SpatialVec> _bodyForces = model->getSystem().getRigidBodyForces(*state, Stage::Instance);
	///// coupling vector
	//model->getMultibodySystem().realize(*state, SimTK::Stage::Instance);
	//shoulder->getCoordinate().setLocked(*state, false);
	//state->updU() = 0;
	//osim_double_adouble tempvalue = 1.0;
	//shoulder->getCoordinate().setSpeedValue(*state, tempvalue);
	//model->getMultibodySystem().realize(*state, SimTK::Stage::Velocity);
	////osim_double_adouble tempvalueeeee = 1.0;
	//Vector couplingVector(state->getNU());
	//for (int i = 0; i < state->getNU(); ++i) {
	//	couplingVector[i] = state->getU()[i] / shoulder->getCoordinate().getSpeedValue(*state);
	//}
	///////// moment arm solve
	//std::cout << "couplingVector pre updU " << couplingVector << std::endl;
	//// state.updU() = 0;
	//state->updU().setToZero();
	//std::cout << "couplingVector post updU " << couplingVector << std::endl;
	//_bodyForces *= 0;
	//Vector _generalizedForces;
	//_generalizedForces = 0;
	//// apply a tension of unity to the bodies of the path
	//std::cout << "couplingVector pre pdmf " << couplingVector << std::endl;
	//Vector pathDependentMobilityForces(state->getNU(), 0.0);
	//std::cout << "couplingVector post pdmf " << couplingVector << std::endl;
	//osim_double_adouble temp_tension = 1.0;
	//onefakemuscle->getGeometryPath().addInEquivalentForces(*state, temp_tension, _bodyForces, pathDependentMobilityForces);
	////std::cout << _bodyForces << std::endl;
	//model->getMultibodySystem().getMatterSubsystem().multiplyBySystemJacobianTranspose(*state, _bodyForces, _generalizedForces);
	//_generalizedForces += pathDependentMobilityForces;
	//std::cout << "generalizedForces " << _generalizedForces << std::endl;
	//std::cout << "_bodyForces " << _bodyForces << std::endl;

	//~couplingVector*_generalizedForces;
	//std::cout << "momentarmshoulder " <<  ~couplingVector*_generalizedForces << std::endl;
	//std::cout << "" << std::endl;


	//std::cout << state->getQ() << std::endl;
	//std::cout << state->getU() << std::endl;

	//// Second call => elbow
	//std::cout << "Second call from scratch" << std::endl;

	//Vector_<SimTK::SpatialVec> bodyForces = model->getSystem().getRigidBodyForces(*state, Stage::Instance);
	///// coupling vector
	//model->getMultibodySystem().realize(*state, SimTK::Stage::Instance);
	//elbow->getCoordinate().setLocked(*state, false);
	//state->updU() = 0;
	//osim_double_adouble _tempvalue = 1.0;
	//elbow->getCoordinate().setSpeedValue(*state, _tempvalue);
	//model->getMultibodySystem().realize(*state, SimTK::Stage::Velocity);
	//Vector _couplingVector(state->getNU());
	//for (int i = 0; i < state->getNU(); ++i) {
	//	_couplingVector[i] = state->getU()[i] / elbow->getCoordinate().getSpeedValue(*state);
	//}
	///////// moment arm solve
	//std::cout << "couplingVector pre updU " << _couplingVector << std::endl;
	//// state.updU() = 0;
	//state->updU().setToZero();
	//std::cout << "couplingVector post updU " << _couplingVector << std::endl;
	//bodyForces *= 0;
	//Vector generalizedForces;
	//generalizedForces = 0;
	//// apply a tension of unity to the bodies of the path
	//std::cout << "couplingVector pre pdmf " << _couplingVector << std::endl;
	//Vector _pathDependentMobilityForces(state->getNU(), 0.0);
	//std::cout << "couplingVector post pdmf " << _couplingVector << std::endl;
	//osim_double_adouble _temp_tension = 1.0;
	//onefakemuscle->getGeometryPath().addInEquivalentForces(*state, _temp_tension, bodyForces, _pathDependentMobilityForces);
	////std::cout << _bodyForces << std::endl;
	//model->getMultibodySystem().getMatterSubsystem().multiplyBySystemJacobianTranspose(*state, bodyForces, generalizedForces);
	//generalizedForces += _pathDependentMobilityForces;
	//std::cout << "generalizedForces " << generalizedForces << std::endl;
	//std::cout << "_bodyForces " << bodyForces << std::endl;
	//std::cout << "_couplingVector " << _couplingVector << std::endl;
	//~_couplingVector*generalizedForces;
	//std::cout << "momentarmelbow " << ~_couplingVector*generalizedForces << std::endl;
	//std::cout << "" << std::endl;


	//adouble y[2];
	//y[0] = residualMobilityForces[0];
	//y[1] = residualMobilityForces[1];

	//double residualMobilityForces_f0;
	//double residualMobilityForces_f1;

	//y[0] >>= residualMobilityForces_f0;
	//y[1] >>= residualMobilityForces_f1;

	//trace_off();

	return 0;
};