/*  This code describes the OpenSim model
    Author: Antoine Falisse
    Contributor: Joris Gillis, Gil Serrancoli, Chris Dembia
*/
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

/*  The function F describes the OpenSim model. F takes as inputs joint
    positions and velocities (states x), and returns several variables for use
    when computing the contact forces in the optimal control problems. F is
    templatized using type T. F(x)->(r).
*/

// Inputs/outputs of function F
/// number of vectors in inputs/outputs of function F
constexpr int n_in = 1;
constexpr int n_out = 1;
/// number of elements in input/output vectors of function F
constexpr int ndof = 21;        // # degrees of freedom (excluding locked)
constexpr int NX = ndof*2;      // # states
constexpr int NR = 84;          // # output variables

// Helper function value
template<typename T>
T value(const Recorder& e) { return e; }
template<>
double value(const Recorder& e) { return e.getValue(); }

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
    OpenSim::CustomJoint* hip_r;
    OpenSim::CustomJoint* hip_l;
    OpenSim::CustomJoint* knee_r;
    OpenSim::CustomJoint* knee_l;
    OpenSim::CustomJoint* ankle_r;
    OpenSim::CustomJoint* ankle_l;
    OpenSim::CustomJoint* subtalar_r;
    OpenSim::CustomJoint* subtalar_l;
    OpenSim::WeldJoint* mtp_r;
    OpenSim::WeldJoint* mtp_l;
    OpenSim::CustomJoint* back;

    // OpenSim model: initialize components
    /// Model
    model = new OpenSim::Model();
    /// Body specifications
    pelvis = new OpenSim::Body("pelvis", 3.69727, Vec3(-0.0737526, -0.0226889, 0), Inertia(0.0238289107, 0.0270444653, 0.029685404, 0, 0, 0));
    femur_l = new OpenSim::Body("femur_l", 4.64393, Vec3(0, -0.159112, 0), Inertia(0.078825664, 0.0161678053, 0.078825664, 0, 0, 0));
    femur_r = new OpenSim::Body("femur_r", 4.64393, Vec3(0, -0.161537, 0), Inertia(0.078825664, 0.0161678053, 0.078825664, 0, 0, 0));
    tibia_l = new OpenSim::Body("tibia_l", 1.43323, Vec3(0, -0.165359, 0), Inertia(0.0132395587, 0.002275956, 0.0137832813, 0, 0, 0));
    tibia_r = new OpenSim::Body("tibia_r", 1.43323, Vec3(0, -0.162479, 0), Inertia(0.0132395587, 0.002275956, 0.0137832813, 0, 0, 0));
    talus_l = new OpenSim::Body("talus_l", 0.01986, Vec3(0.00464602, 0.00194288, 0), Inertia(1.324e-006, 4.413e-007, 1.324e-006, 0, 0, 0));
    talus_r = new OpenSim::Body("talus_r", 0.01986, Vec3(0.00459638, 0.00192212, 0), Inertia(1.324e-006, 4.413e-007, 1.324e-006, 0, 0, 0));
    calcn_l = new OpenSim::Body("calcn_l", 0.39058, Vec3(0.0860486, 0.0131778, 0), Inertia(0.000401172, 0.0010212453, 0.0011840973, 0, 0, 0));
    calcn_r = new OpenSim::Body("calcn_r", 0.39058, Vec3(0.085129, 0.013037, 0), Inertia(0.000401172, 0.0010212453, 0.0011840973, 0, 0, 0));
    toes_l = new OpenSim::Body("toes_l", 0.04303, Vec3(0.0259333, -0.0021963, -0.00887), Inertia(4.41333e-005, 0.00011254, 0.000129752, 0, 0, 0));
    toes_r = new OpenSim::Body("toes_r", 0.04303, Vec3(0.0256561, -0.00217283, 0.008775), Inertia(4.41333e-005, 0.00011254, 0.000129752, 0, 0, 0));
    torso = new OpenSim::Body("torso", 16.25541, Vec3(0.0123281, 0.218025, 0), Inertia(0.215542616846467, 0.135378536674077, 0.291668449684595, 0, 0, 0));
    /// Joints
    /// Ground-Pelvis transform
    SpatialTransform st_ground_pelvis;
    st_ground_pelvis[0].setCoordinateNames(OpenSim::Array<std::string>("lower_torso_RX", 1, 1));
    st_ground_pelvis[0].setFunction(new LinearFunction());
    st_ground_pelvis[0].setAxis(Vec3(1, 0, 0));
    st_ground_pelvis[1].setCoordinateNames(OpenSim::Array<std::string>("lower_torso_RY", 1, 1));
    st_ground_pelvis[1].setFunction(new LinearFunction());
    st_ground_pelvis[1].setAxis(Vec3(0, 1, 0));
    st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("lower_torso_RZ", 1, 1));
    st_ground_pelvis[2].setFunction(new LinearFunction());
    st_ground_pelvis[2].setAxis(Vec3(0, 0, 1));
    st_ground_pelvis[3].setCoordinateNames(OpenSim::Array<std::string>("lower_torso_TX", 1, 1));
    st_ground_pelvis[3].setFunction(new LinearFunction());
    st_ground_pelvis[3].setAxis(Vec3(1, 0, 0));
    st_ground_pelvis[4].setCoordinateNames(OpenSim::Array<std::string>("lower_torso_TY", 1, 1));
    st_ground_pelvis[4].setFunction(new LinearFunction());
    st_ground_pelvis[4].setAxis(Vec3(0, 1, 0));
    st_ground_pelvis[5].setCoordinateNames(OpenSim::Array<std::string>("lower_torso_TZ", 1, 1));
    st_ground_pelvis[5].setFunction(new LinearFunction());
    st_ground_pelvis[5].setAxis(Vec3(0, 0, 1));
    /// Hip_l transform
    SpatialTransform st_hip_l;
    st_hip_l[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_l", 1, 1));
    st_hip_l[0].setFunction(new LinearFunction());
    st_hip_l[0].setAxis(Vec3(0, 0, 1));
    st_hip_l[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_l", 1, 1));
    st_hip_l[1].setFunction(new LinearFunction());
    st_hip_l[1].setAxis(Vec3(-1, 0, 0));
    st_hip_l[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_l", 1, 1));
    st_hip_l[2].setFunction(new LinearFunction());
    st_hip_l[2].setAxis(Vec3(0, -1, 0));
    /// Hip_r transform
    SpatialTransform st_hip_r;
    st_hip_r[0].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion_r", 1, 1));
    st_hip_r[0].setFunction(new LinearFunction());
    st_hip_r[0].setAxis(Vec3(0, 0, 1));
    st_hip_r[1].setCoordinateNames(OpenSim::Array<std::string>("hip_adduction_r", 1, 1));
    st_hip_r[1].setFunction(new LinearFunction());
    st_hip_r[1].setAxis(Vec3(1, 0, 0));
    st_hip_r[2].setCoordinateNames(OpenSim::Array<std::string>("hip_rotation_r", 1, 1));
    st_hip_r[2].setFunction(new LinearFunction());
    st_hip_r[2].setAxis(Vec3(0, 1, 0));
    /// Knee_l transform
    SpatialTransform st_knee_l;
    st_knee_l[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_l", 1, 1));
    st_knee_l[2].setFunction(new LinearFunction());
    st_knee_l[2].setAxis(Vec3(0, 0, -1));
    /// Knee_r transform
    SpatialTransform st_knee_r;
    st_knee_r[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle_r", 1, 1));
    st_knee_r[2].setFunction(new LinearFunction());
    st_knee_r[2].setAxis(Vec3(0, 0, -1));
    /// Ankle_l transform
    SpatialTransform st_ankle_l;
    st_ankle_l[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_l", 1, 1));
    st_ankle_l[0].setFunction(new LinearFunction());
    st_ankle_l[0].setAxis(Vec3(0.104529047112, 0.173649078266, 0.979244441356));
    /// Ankle_r transform
    SpatialTransform st_ankle_r;
    st_ankle_r[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle_r", 1, 1));
    st_ankle_r[0].setFunction(new LinearFunction());
    st_ankle_r[0].setAxis(Vec3(-0.104529047112, -0.173649078266, 0.979244441356));
    /// Subtalar_l transform
    SpatialTransform st_subtalar_l;
    st_subtalar_l[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_l", 1, 1));
    st_subtalar_l[0].setFunction(new LinearFunction());
    st_subtalar_l[0].setAxis(Vec3(-0.787180020856, -0.604747016023, -0.120949003205));
    /// Subtalar_r transform
    SpatialTransform st_subtalar_r;
    st_subtalar_r[0].setCoordinateNames(OpenSim::Array<std::string>("subtalar_angle_r", 1, 1));
    st_subtalar_r[0].setFunction(new LinearFunction());
    st_subtalar_r[0].setAxis(Vec3(0.787180020856, 0.604747016023, -0.120949003205));
    /// Back transform
    SpatialTransform st_back;
    st_back[0].setCoordinateNames(OpenSim::Array<std::string>("lumbar_extension", 1, 1));
    st_back[0].setFunction(new LinearFunction());
    st_back[0].setAxis(Vec3(0, 0, 1));
    st_back[1].setCoordinateNames(OpenSim::Array<std::string>("lumbar_bending", 1, 1));
    st_back[1].setFunction(new LinearFunction());
    st_back[1].setAxis(Vec3(1, 0, 0));
    st_back[2].setCoordinateNames(OpenSim::Array<std::string>("lumbar_rotation", 1, 1));
    st_back[2].setFunction(new LinearFunction());
    st_back[2].setAxis(Vec3(0, 1, 0));
    /// Joint specifications
    ground_pelvis = new CustomJoint("ground_pelvis", model->getGround(), Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0), st_ground_pelvis);
    hip_l = new CustomJoint("hip_l", *pelvis, Vec3(-0.0384699, -0.0708953, -0.0646186), Vec3(0), *femur_l, Vec3(0), Vec3(0), st_hip_l);
    hip_r = new CustomJoint("hip_r", *pelvis, Vec3(-0.0443968, -0.0673054, 0.0671603), Vec3(0), *femur_r, Vec3(0), Vec3(0), st_hip_r);
    knee_l = new CustomJoint("knee_l", *femur_l, Vec3(0.00000009, -0.393218, 0), Vec3(0), *tibia_l, Vec3(0), Vec3(0.105096325674, 0.048462535877, -0.005110024376), st_knee_l);
    knee_r = new CustomJoint("knee_r", *femur_r, Vec3(0.0000001, -0.394719, -0.00000002), Vec3(0), *tibia_r, Vec3(0), Vec3(-0.064206849712, -0.022848068757, -0.001468892896), st_knee_r);
    ankle_l = new CustomJoint("ankle_l", *tibia_l, Vec3(0, -0.386280874342227, 0), Vec3(0), *talus_l, Vec3(0), Vec3(0), st_ankle_l);
    ankle_r = new CustomJoint("ankle_r", *tibia_r, Vec3(0, -0.386028856205262, 0), Vec3(0), *talus_r, Vec3(0), Vec3(0), st_ankle_r);
    subtalar_l = new CustomJoint("subtalar_l", *talus_l, Vec3(-0.041198, -0.035436, -0.00669), Vec3(0), *calcn_l, Vec3(0), Vec3(0),st_subtalar_l);
    subtalar_r = new CustomJoint("subtalar_r", *talus_r, Vec3(-0.040757, -0.035058, 0.006619), Vec3(0), *calcn_r, Vec3(0), Vec3(0),st_subtalar_r);
    mtp_l = new WeldJoint("mtp_l", *calcn_l, Vec3(0.149349, -0.001689, -0.000912), Vec3(0), *toes_l, Vec3(0), Vec3(0));
    mtp_r = new WeldJoint("mtp_r", *calcn_r, Vec3(0.147753, -0.001671, 0.000903), Vec3(0), *toes_r, Vec3(0), Vec3(0));
    back = new CustomJoint("back", *pelvis, Vec3(-0.072018, 0.029113, 00), Vec3(0), *torso, Vec3(0), Vec3(0), st_back);
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

    // Initialize system and state
    SimTK::State* state;
    state = new State(model->initSystem());

    // Read inputs
    std::vector<T> x(arg[0], arg[0] + NX);

    // States and controls
    Vector QsUs(NX); /// joint positions (Qs) and velocities (Us) - states

    // Assign inputs to model variables
    /// States
    for (int i = 0; i < NX; ++i) QsUs[i] = x[i];

    // Set state variables and realize
    model->setStateVariableValues(*state, QsUs);
    model->realizeVelocity(*state);

    // Get position, velocity and transform of calcn and toes
    SpatialVec vel_calcn_l =  calcn_l->getVelocityInGround(*state);
    SpatialVec vel_calcn_r =  calcn_r->getVelocityInGround(*state);
    Vec3 pos_calcn_l = calcn_l->getPositionInGround(*state);
    Vec3 pos_calcn_r = calcn_r->getPositionInGround(*state);
    Transform TR_GB_calcn_l = calcn_l->getMobilizedBody().getBodyTransform(*state);
    Transform TR_GB_calcn_r = calcn_r->getMobilizedBody().getBodyTransform(*state);

    SpatialVec vel_toes_l =  toes_l->getVelocityInGround(*state);
    SpatialVec vel_toes_r =  toes_r->getVelocityInGround(*state);
    Vec3 pos_toes_l = toes_l->getPositionInGround(*state);
    Vec3 pos_toes_r = toes_r->getPositionInGround(*state);
    Transform TR_GB_toes_l = toes_l->getMobilizedBody().getBodyTransform(*state);
    Transform TR_GB_toes_r = toes_r->getMobilizedBody().getBodyTransform(*state);

    // Extract results
    int nc = 3;
    for (int i = 0; i < nc; ++i) {
        res[0][i] = value<T>(vel_calcn_l[0][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i+nc] = value<T>(vel_calcn_l[1][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i+nc+nc] = value<T>(vel_calcn_r[0][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i+nc+nc+nc] = value<T>(vel_calcn_r[1][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i+nc+nc+nc+nc] = value<T>(pos_calcn_l[i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][i+nc+nc+nc+nc+nc] = value<T>(pos_calcn_r[i]);
    }
    int count = 0;
    for (int i = 0; i < nc; ++i) {
        for (int j = 0; j < nc; ++j) {
            res[0][nc+nc+nc+nc+nc+nc+count] = TR_GB_calcn_l.R().get(i,j);
            ++ count;
        }
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+i] = TR_GB_calcn_l.T().get(i);
    }
    int count2 = 0;
    for (int i = 0; i < nc; ++i) {
        for (int j = 0; j < nc; ++j) {
            res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+count2] = TR_GB_calcn_r.R().get(i,j);
            ++ count2;
        }
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+i] = TR_GB_calcn_r.T().get(i);
    }

    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+i] = value<T>(vel_toes_l[0][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+i+nc] = value<T>(vel_toes_l[1][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+i+nc+nc] = value<T>(vel_toes_r[0][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+i+nc+nc+nc] = value<T>(vel_toes_r[1][i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+i+nc+nc+nc+nc] = value<T>(pos_toes_l[i]);
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+i+nc+nc+nc+nc+nc] = value<T>(pos_toes_r[i]);
    }
    int count3 = 0;
    for (int i = 0; i < nc; ++i) {
        for (int j = 0; j < nc; ++j) {
            res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+nc+nc+nc+nc+nc+nc+count3] = TR_GB_toes_l.R().get(i,j);
            ++ count3;
        }
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+nc+nc+nc+nc+nc+nc+nc*nc+i] = TR_GB_toes_l.T().get(i);
    }
    int count4 = 0;
    for (int i = 0; i < nc; ++i) {
        for (int j = 0; j < nc; ++j) {
            res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+nc+nc+nc+nc+nc+nc+nc*nc+nc+count4] = TR_GB_toes_r.R().get(i,j);
            ++ count4;
        }
    }
    for (int i = 0; i < nc; ++i) {
        res[0][nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+nc+nc+nc+nc+nc+nc+nc+nc*nc+nc+nc*nc+i] = TR_GB_toes_r.T().get(i);
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
    Recorder tau[NR];

    for (int i = 0; i < NX; ++i) x[i] <<= 0;

    const Recorder* Recorder_arg[n_in] = { x };
    Recorder* Recorder_res[n_out] = { tau };

    F_generic<Recorder>(Recorder_arg, Recorder_res);

    double res[NR];
    for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

    Recorder::stop_recording();

    return 0;

}
