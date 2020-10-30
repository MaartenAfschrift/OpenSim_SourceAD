/*  This code describes the OpenSim model and the skeleton dynamics
    Author: Antoine Falisse
    Contributor: Joris Gillis, Gil Serrancoli, Chris Dembia
*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>
#include "SimTKcommon/internal/recorder.h"

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>

using namespace SimTK;
using namespace OpenSim;

/*  The function F describes the OpenSim model and, implicitly, the skeleton
    dynamics. F takes as inputs joint positions and velocities (states x),
    joint accelerations (controls u), and returns the joint torques as well as
    several variables for use in the optimal control problems. F is templatized
    using type T. F(x,u)->(r).
*/

// Inputs/outputs of function F
/// number of vectors in inputs/outputs of function F
constexpr int n_in = 2;
constexpr int n_out = 1;
/// number of elements in input/output vectors of function F
constexpr int ndof = 11;    // # degrees of freedom
constexpr int NX = 2*ndof;  // # states
constexpr int NU = ndof;    // # controls
constexpr int NR = ndof;  //

// Helper function value
template<typename T>
T value(const Recorder& e) { return e; }
template<>
double value(const Recorder& e) { return e.getValue(); }

// OpenSim and Simbody use different indices for the states/controls when the
// kinematic chain has joints up and down the origin (e.g., lumbar joint/arms
// and legs with pelvis as origin).
// The two following functions allow getting the indices from one reference
// system to the other. These functions are inspired from
// createSystemYIndexMap() in Moco.
// getIndicesOSInSimbody() returns the indices of the OpenSim Qs in the Simbody
// reference system. Note that we only care about the order here so we divide
// by 2 because the states include both Qs and Qdots.
SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) {
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    SimTK::Array_<int> idxOSInSimbody(s.getNQ());
    s.updQ() = 0;
    for (int iy = 0; iy < s.getNQ(); ++iy) {
        s.updQ()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                s.updQ()[iy] = 0;
                idxOSInSimbody[iy] = isv/2;
                break;
            }
        }
    }
    return idxOSInSimbody;
}
// getIndicesSimbodyInOS() returns the indices of the Simbody Qs in the OpenSim
// reference system.
SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) {
    auto idxOSInSimbody = getIndicesOSInSimbody(model);
    auto s = model.getWorkingState();
    SimTK::Array_<int> idxSimbodyInOS(s.getNQ());
	for (int iy = 0; iy < s.getNQ(); ++iy) {
		for (int iyy = 0; iyy < s.getNQ(); ++iyy) {
			if (idxOSInSimbody[iyy] == iy) {
				idxSimbodyInOS[iy] = iyy;
				break;
			}
		}
	}
    return idxSimbodyInOS;
}

// Function F
template<typename T>
int F_generic(const T** arg, T** res) {

    // OpenSim model: create components
    /// Model
    OpenSim::Model* model;
    /// Bodies
    OpenSim::Body* pelvis;
    OpenSim::Body* femur_r;
    OpenSim::Body* femur_l;
    OpenSim::Body* tibia_r;
    OpenSim::Body* tibia_l;
    OpenSim::Body* talus_r;
    OpenSim::Body* talus_l;
    OpenSim::Body* calcn_r;
    OpenSim::Body* calcn_l;
    OpenSim::Body* toes_r;
    OpenSim::Body* toes_l;
    OpenSim::Body* torso;
    /// Joints
    OpenSim::CustomJoint* ground_pelvis;
    OpenSim::PinJoint* hip_r;
    OpenSim::PinJoint* hip_l;
    OpenSim::PinJoint* knee_r;
    OpenSim::PinJoint* knee_l;
    OpenSim::PinJoint* ankle_r;
    OpenSim::PinJoint* ankle_l;
    OpenSim::WeldJoint* subtalar_r;
    OpenSim::WeldJoint* subtalar_l;
    OpenSim::PinJoint* mtp_r;
    OpenSim::PinJoint* mtp_l;
    OpenSim::WeldJoint* back;
    /// Contact elements
    OpenSim::HuntCrossleyForce_smooth* HC_s1_r;
    OpenSim::HuntCrossleyForce_smooth* HC_s2_r;
    OpenSim::HuntCrossleyForce_smooth* HC_s3_r;
    OpenSim::HuntCrossleyForce_smooth* HC_s1_l;
    OpenSim::HuntCrossleyForce_smooth* HC_s2_l;
    OpenSim::HuntCrossleyForce_smooth* HC_s3_l;

    // OpenSim model: initialize components
    /// Model
    model = new OpenSim::Model();
    /// Body specifications
    pelvis = new OpenSim::Body("pelvis", 9.7143336091724048, Vec3(-0.068431260352111167, 0, 0), Inertia(0.084795236055270728, 0.071844990860059146, 0.047759184509729331, 0, 0, 0));
    femur_l = new OpenSim::Body("femur_l", 7.6723191502382786, Vec3(0, -0.16734021487797751, 0), Inertia(0.10701920433197534, 0.028053577834595483, 0.11285370912378581, 0, 0, 0));
    femur_r = new OpenSim::Body("femur_r", 7.6723191502382786, Vec3(0, -0.16734021487797751, 0), Inertia(0.10701920433197534, 0.028053577834595483, 0.11285370912378581, 0, 0, 0));
    tibia_l = new OpenSim::Body("tibia_l", 3.0581550357482121, Vec3(0, -0.18635251395796504, 0), Inertia(0.041418155207686803, 0.004191122848396879, 0.041993407363349118, 0, 0, 0));
    tibia_r = new OpenSim::Body("tibia_r", 3.0581550357482121, Vec3(0, -0.18635251395796504, 0), Inertia(0.041418155207686803, 0.004191122848396879, 0.041993407363349118, 0, 0, 0));
    talus_l = new OpenSim::Body("talus_l", 0.082485638186061028, Vec3(0), Inertia(0.00076310987643225315, 0.00076310987643225315, 0.00076310987643225315, 0, 0, 0));
    talus_r = new OpenSim::Body("talus_r", 0.082485638186061028, Vec3(0), Inertia(0.00076310987643225315, 0.00076310987643225315, 0.00076310987643225315, 0, 0, 0));
    calcn_l = new OpenSim::Body("calcn_l", 1.0310704773257626, Vec3(0.096184339663296911, 0.028855301898989071, 0), Inertia(0.0010683538270051544, 0.0029761285180857871, 0.003128750493372238, 0, 0, 0));
    calcn_r = new OpenSim::Body("calcn_r", 1.0310704773257626, Vec3(0.096184339663296911, 0.028855301898989071, 0), Inertia(0.0010683538270051544, 0.0029761285180857871, 0.003128750493372238, 0, 0, 0));
    toes_l = new OpenSim::Body("toes_l", 0.17866389231100815, Vec3(0.03327978152350073, 0.0057710603797978145, 0.016832259441076958), Inertia(7.631098764322532e-05, 0.00015262197528645064, 7.631098764322532e-05, 0, 0, 0));
    toes_r = new OpenSim::Body("toes_r", 0.17866389231100815, Vec3(0.03327978152350073, 0.0057710603797978145, -0.016832259441076958), Inertia(7.631098764322532e-05, 0.00015262197528645064, 7.631098764322532e-05, 0, 0, 0));
    torso = new OpenSim::Body("torso", 28.240278003208967, Vec3(-0.028926628525507352, 0.308550704272078427, 0), Inertia(1.1307751170491251, 0.57938324918997219, 1.0977222804639659, 0, 0, 0));
    /// Joints
    /// Ground_pelvis transform
    SpatialTransform st_ground_pelvis;
    st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1));
    st_ground_pelvis[0].setFunction(new LinearFunction());
    st_ground_pelvis[0].setAxis(Vec3(0,0,1));
    st_ground_pelvis[1].setFunction(new Constant(0));
    st_ground_pelvis[1].setAxis(Vec3(1,0,0));
    st_ground_pelvis[2].setFunction(new Constant(0));
    st_ground_pelvis[2].setAxis(Vec3(0,1,0));
    st_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tx", 1, 1));
    st_ground_pelvis[3].setFunction(new LinearFunction());
    st_ground_pelvis[3].setAxis(Vec3(1,0,0));
    st_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("pelvis_ty", 1, 1));
    st_ground_pelvis[4].setFunction(new LinearFunction());
    st_ground_pelvis[4].setAxis(Vec3(0,1,0));
    st_ground_pelvis[5].setFunction(new LinearFunction());
    st_ground_pelvis[5].setAxis(Vec3(0,0,1));
	/// Hip_l transform
	SpatialTransform st_hip_l;
	st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
	st_hip_l[2].setFunction(new LinearFunction());
	st_hip_l[2].setAxis(Vec3(0, 0, 1));
	/// Hip_r transform
	SpatialTransform st_hip_r;
	st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
	st_hip_r[2].setFunction(new LinearFunction());
	st_hip_r[2].setAxis(Vec3(0, 0, 1));
	/// Knee_l transform
	SpatialTransform st_knee_l;
	st_knee_l[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
	st_knee_l[2].setFunction(new LinearFunction());
	st_knee_l[2].setAxis(Vec3(0, 0, 1));
	/// Knee_r transform
	SpatialTransform st_knee_r;
	st_knee_r[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
	st_knee_r[2].setFunction(new LinearFunction());
	st_knee_r[2].setAxis(Vec3(0, 0, 1));
   
    /// Joint specifications
    ground_pelvis = new CustomJoint("ground_pelvis", model->getGround(), Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0), st_ground_pelvis);
    hip_l = new PinJoint("hip_l", *pelvis, Vec3(-0.068431260352111167, -0.063978872832737607, -0.082439592243860341), Vec3(0), *femur_l, Vec3(0), Vec3(0));
    hip_r = new PinJoint("hip_r", *pelvis, Vec3(-0.068431260352111167, -0.063978872832737607, 0.082439592243860341), Vec3(0), *femur_r, Vec3(0), Vec3(0));
	knee_l = new PinJoint("knee_l", *femur_l, Vec3(-0.00451221232146798, -0.396907245921447, 0), Vec3(0), *tibia_l, Vec3(0), Vec3(0));
	knee_r = new PinJoint("knee_r", *femur_r, Vec3(-0.00451221232146798, -0.396907245921447, 0), Vec3(0), *tibia_r, Vec3(0), Vec3(0));
    ankle_l = new PinJoint("ankle_l", *tibia_l, Vec3(0, -0.42919968399531311, 0), Vec3(0), *talus_l, Vec3(0), Vec3(0));
    ankle_r = new PinJoint("ankle_r", *tibia_r, Vec3(0, -0.42919968399531311, 0), Vec3(0), *talus_r, Vec3(0), Vec3(0));
    subtalar_l = new WeldJoint("subtalar_l", *talus_l, Vec3(-0.046909102453789903, -0.040349330488753055, -0.0076177997013331146), Vec3(1.7681899999999999, -0.906223, 1.8196000000000001), *calcn_l, Vec3(0), Vec3(1.7681899999999999, -0.906223, 1.8196000000000001));
    subtalar_r = new WeldJoint("subtalar_r", *talus_r, Vec3(-0.046909102453789903, -0.040349330488753055, 0.0076177997013331146), Vec3(-1.7681899999999999, 0.906223, 1.8196000000000001), *calcn_r, Vec3(0), Vec3(-1.7681899999999999, 0.906223, 1.8196000000000001));
    mtp_l = new PinJoint("mtp_l", *calcn_l, Vec3(0.17197759931797485, -0.0019236867932659382, -0.0010387908683636067), Vec3(0), *toes_l, Vec3(0), Vec3(0));
    mtp_r = new PinJoint("mtp_r", *calcn_r, Vec3(0.17197759931797485, -0.0019236867932659382, 0.0010387908683636067), Vec3(0), *toes_r, Vec3(0), Vec3(0));
    back = new WeldJoint("back", *pelvis, Vec3(-0.097468570261069226, 0.078884691919336072, 0), Vec3(0), *torso, Vec3(0), Vec3(0));
	
	/// Add bodies and joints to model
	model->addBody(pelvis);		    model->addJoint(ground_pelvis);
	model->addBody(femur_l);		model->addJoint(hip_l);
	model->addBody(femur_r);		model->addJoint(hip_r);
	model->addBody(tibia_l);		model->addJoint(knee_l);
	model->addBody(tibia_r);		model->addJoint(knee_r);
	model->addBody(talus_l);		model->addJoint(ankle_l);
	model->addBody(talus_r);		model->addJoint(ankle_r);
	model->addBody(calcn_l);		model->addJoint(subtalar_l);
	model->addBody(calcn_r);		model->addJoint(subtalar_r);
	model->addBody(toes_l);		    model->addJoint(mtp_l);
	model->addBody(toes_r);		    model->addJoint(mtp_r);
	model->addBody(torso);          model->addJoint(back);

    /// Contact elements
    /// Parameters
    osim_double_adouble radiusSphere_s1 = 0.035;
    osim_double_adouble radiusSphere_s2 = 0.030;
    osim_double_adouble radiusSphere_s3 = 0.025;
    osim_double_adouble stiffness_s1 = 2000000;
    osim_double_adouble stiffness_s2 = 2000000;
    osim_double_adouble stiffness_s3 = 2000000;
    osim_double_adouble dissipation = 2.0;
    osim_double_adouble staticFriction = 0.8;
    osim_double_adouble dynamicFriction = 0.8;
    osim_double_adouble viscousFriction = 0.5;
    osim_double_adouble transitionVelocity = 0.2;
    Vec3 locSphere_s1 = Vec3(0.004, 0.0, 0.0);
    Vec3 locSphere_s2 = Vec3(0.0, 0.0, 0.0);
    Vec3 locSphere_s3 = Vec3(0.05, 0.0, 0.0);
    Vec3 normal = Vec3(0, 1, 0);
    osim_double_adouble offset = 0;
    /// Right foot contact shere specifications
    HC_s1_r = new HuntCrossleyForce_smooth("sphere_s1_r", "calcn_r", locSphere_s1, radiusSphere_s1,
        stiffness_s1, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_s2_r = new HuntCrossleyForce_smooth("sphere_s2_r", "toes_r", locSphere_s2, radiusSphere_s2,
        stiffness_s2, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_s3_r = new HuntCrossleyForce_smooth("sphere_s3_r", "toes_r", locSphere_s3, radiusSphere_s3,
        stiffness_s3, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    /// Add right foot contact spheres to model
    model->addComponent(HC_s1_r);
    HC_s1_r->connectSocket_body_sphere(*calcn_r);
    model->addComponent(HC_s2_r);
    HC_s2_r->connectSocket_body_sphere(*toes_r);
    model->addComponent(HC_s3_r);
    HC_s3_r->connectSocket_body_sphere(*toes_r);
    /// Left foot contact shere specifications
    HC_s1_l = new HuntCrossleyForce_smooth("sphere_s1_l", "calcn_l", locSphere_s1, radiusSphere_s1,
        stiffness_s1, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_s2_l = new HuntCrossleyForce_smooth("sphere_s2_l", "toes_l", locSphere_s2, radiusSphere_s2,
        stiffness_s2, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    HC_s3_l = new HuntCrossleyForce_smooth("sphere_s3_l", "toes_l", locSphere_s3, radiusSphere_s3,
        stiffness_s3, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
    /// Add left foot contact spheres to model
    model->addComponent(HC_s1_l);
    HC_s1_l->connectSocket_body_sphere(*calcn_l);
    model->addComponent(HC_s2_l);
    HC_s2_l->connectSocket_body_sphere(*toes_l);
    model->addComponent(HC_s3_l);
    HC_s3_l->connectSocket_body_sphere(*toes_l);

    // Initialize system and state.
    SimTK::State* state;
    state = new State(model->initSystem());

    // Read inputs
    std::vector<T> x(arg[0], arg[0] + NX);
    std::vector<T> u(arg[1], arg[1] + NU);

    // States and controls
    T ua[NU]; /// joint accelerations (Qdotdots) - controls
    Vector QsUs(NX); /// joint positions (Qs) and velocities (Us) - states

    // Assign inputs to model variables
    /// States
    QsUs.setToZero();
    for (int i = 0; i < NX; ++i) QsUs[i] = x[i];
    /// Controls
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesOSInSimbody = getIndicesOSInSimbody(*model);
    for (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];

    // Set state variables and realize
    model->setStateVariableValues(*state, QsUs);
    model->realizeVelocity(*state);

    // Compute residual forces
    /// appliedMobilityForces (# mobilities)
    Vector appliedMobilityForces(ndof);
    appliedMobilityForces.setToZero();
    /// appliedBodyForces (# bodies + ground)
    Vector_<SpatialVec> appliedBodyForces;
    int nbodies = model->getBodySet().getSize() + 1;
    appliedBodyForces.resize(nbodies);
    appliedBodyForces.setToZero();
    /// Set gravity
    Vec3 gravity(0);
    gravity[1] = -9.80665;
    /// Add weights to appliedBodyForces
    for (int i = 0; i < model->getBodySet().getSize(); ++i) {
        model->getMatterSubsystem().addInStationForce(*state,
            model->getBodySet().get(i).getMobilizedBodyIndex(),
            model->getBodySet().get(i).getMassCenter(),
            model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
    }
    /// Add contact forces to appliedBodyForces
    /// Right foot
    Array<osim_double_adouble> Force_values_s1_r = HC_s1_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_s2_r = HC_s2_r->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_s3_r = HC_s3_r->getRecordValues(*state);
    SpatialVec GRF_s1_r;
    GRF_s1_r[0] = Vec3(Force_values_s1_r[9], Force_values_s1_r[10], Force_values_s1_r[11]);
    GRF_s1_r[1] = Vec3(Force_values_s1_r[6], Force_values_s1_r[7], Force_values_s1_r[8]);
    SpatialVec GRF_s2_r;
    GRF_s2_r[0] = Vec3(Force_values_s2_r[9], Force_values_s2_r[10], Force_values_s2_r[11]);
    GRF_s2_r[1] = Vec3(Force_values_s2_r[6], Force_values_s2_r[7], Force_values_s2_r[8]);
    SpatialVec GRF_s3_r;
    GRF_s3_r[0] = Vec3(Force_values_s3_r[9], Force_values_s3_r[10], Force_values_s3_r[11]);
    GRF_s3_r[1] = Vec3(Force_values_s3_r[6], Force_values_s3_r[7], Force_values_s3_r[8]);
    int nCalcn_r = model->getBodySet().get("calcn_r").getMobilizedBodyIndex();
    int nToes_r = model->getBodySet().get("toes_r").getMobilizedBodyIndex();
    appliedBodyForces[nCalcn_r] = appliedBodyForces[nCalcn_r] + GRF_s1_r;
    appliedBodyForces[nToes_r] = appliedBodyForces[nToes_r] + GRF_s2_r + GRF_s3_r;
    /// Left foot
    Array<osim_double_adouble> Force_values_s1_l = HC_s1_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_s2_l = HC_s2_l->getRecordValues(*state);
    Array<osim_double_adouble> Force_values_s3_l = HC_s3_l->getRecordValues(*state);
    SpatialVec GRF_s1_l;
    GRF_s1_l[0] = Vec3(Force_values_s1_l[9], Force_values_s1_l[10], Force_values_s1_l[11]);
    GRF_s1_l[1] = Vec3(Force_values_s1_l[6], Force_values_s1_l[7], Force_values_s1_l[8]);
    SpatialVec GRF_s2_l;
    GRF_s2_l[0] = Vec3(Force_values_s2_l[9], Force_values_s2_l[10], Force_values_s2_l[11]);
    GRF_s2_l[1] = Vec3(Force_values_s2_l[6], Force_values_s2_l[7], Force_values_s2_l[8]);
    SpatialVec GRF_s3_l;
    GRF_s3_l[0] = Vec3(Force_values_s3_l[9], Force_values_s3_l[10], Force_values_s3_l[11]);
    GRF_s3_l[1] = Vec3(Force_values_s3_l[6], Force_values_s3_l[7], Force_values_s3_l[8]);
    int nCalcn_l = model->getBodySet().get("calcn_l").getMobilizedBodyIndex();
    int nToes_l = model->getBodySet().get("toes_l").getMobilizedBodyIndex();
    appliedBodyForces[nCalcn_l] = appliedBodyForces[nCalcn_l] + GRF_s1_l;
    appliedBodyForces[nToes_l] = appliedBodyForces[nToes_l] + GRF_s2_l + GRF_s3_l;
    /// knownUdot
    Vector knownUdot(ndof);
    knownUdot.setToZero();
    for (int i = 0; i < ndof; ++i) knownUdot[i] = ua[i];
    /// Calculate residual forces
    Vector residualMobilityForces(ndof);
    residualMobilityForces.setToZero();
    model->getMatterSubsystem().calcResidualForceIgnoringConstraints(
        *state, appliedMobilityForces, appliedBodyForces, knownUdot,
        residualMobilityForces);

    // Extract results
    /// Residual forces
    /// OpenSim and Simbody have different state orders so we need to adjust
    auto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);
    for (int i = 0; i < NU; ++i) {
            res[0][i] = value<T>(residualMobilityForces[indicesSimbodyInOS[i]]);
    }

    return 0;

}

/* In main(), the Recorder is used to save the expression graph of function F.
This expression graph is saved as a MATLAB function named foo.m. From this
function, a c-code can be generated via CasADi and then compiled as a dll. This
dll is then imported in MATLAB as an external function. With this workflow,
CasADi can use algorithmic differentiation to differentiate the function F.
*/
int main() {

    Recorder x[NX];
    Recorder u[NU];
    Recorder tau[NR];

    for (int i = 0; i < NX; ++i) x[i] <<= 0;
    for (int i = 0; i < NU; ++i) u[i] <<= 0;

    const Recorder* Recorder_arg[n_in] = { x,u };
    Recorder* Recorder_res[n_out] = { tau };

    F_generic<Recorder>(Recorder_arg, Recorder_res);

    double res[NR];
    for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

    Recorder::stop_recording();

    return 0;
}