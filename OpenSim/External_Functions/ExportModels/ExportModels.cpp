/*  This code describes the OpenSim model and the skeleton dynamics
    Author: Antoine Falisse
    Contributor: Joris Gillis, Gil Serrancoli, Chris Dembia
*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PlanarJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce_smooth.h>

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>
#include <cmath>


using namespace SimTK;
using namespace OpenSim;



int main() {


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
	OpenSim::PlanarJoint* ground_pelvis;
	OpenSim::CustomJoint* hip_r;
	OpenSim::CustomJoint* hip_l;
	OpenSim::CustomJoint* knee_r;
	OpenSim::CustomJoint* knee_l;
	OpenSim::PinJoint* ankle_r;
	OpenSim::PinJoint* ankle_l;
	OpenSim::WeldJoint* subtalar_r;
	OpenSim::WeldJoint* subtalar_l;
	OpenSim::WeldJoint* mtp_r;
	OpenSim::WeldJoint* mtp_l;
	OpenSim::PinJoint* back;
	/// Contact elements
	OpenSim::HuntCrossleyForce_smooth* HC_heel_r;
	OpenSim::HuntCrossleyForce_smooth* HC_front_r;
	OpenSim::HuntCrossleyForce_smooth* HC_heel_l;
	OpenSim::HuntCrossleyForce_smooth* HC_front_l;


	// Inertia of the exoskeleton
	/// Human segment properties
	Vec3 COMTibia = Vec3(0, -0.180489, 0);
	Vec3 COMTalus = Vec3(0, 0, 0);
	double  lTibia = 0.39;
	double mTibia = 3.05815503574821;
	double mTalus = 0.082485638186061;
	Vec3 ITibia = Vec3(0.03885270, 0.0039315, 0.0393923);
	Vec3 ITalus = Vec3(0.0006890, 0.0006890, 0.0006890);

	/// Ankle foot exoskeleton properties
	double mShank_Exo = 0.88*0.5;
	double mFoot_Exo = 0.88*0.5;
	Vec3 COMShank_Exo = Vec3(-0.0381, -lTibia + 0.1016, 0);
	Vec3 COMFoot_Exo = Vec3(0, -0.0381, 0);
	Vec3 IFootExo = Vec3(0.0021, 0.0050, 0.0068);
	Vec3 IShankExo = Vec3(0.0073, 0.0066, 0.0027);

	/// Get combined COM location
	double mTib_tot = mTibia + mShank_Exo;
	double mFoot_tot = mTalus + mFoot_Exo;
	Vec3 COM_TibNew, COM_FootNew;
	double COMs;
	for (int i = 0; i < 3; ++i) {
		COMs = (COMShank_Exo.get(i)*mShank_Exo + COMTibia.get(i)*mTibia) / mTib_tot;
		COM_TibNew.set(i, COMs);
		COMs = (COMFoot_Exo.get(i)*mFoot_Exo + COMTalus.get(i)*mTalus) / mTib_tot;
		COM_FootNew.set(i, COMs);
	}

	// Get the new inertia: x -axis	
	double dTibia, dExoShank, dTalus, dExoFoot;
	dTibia = sqrt(pow(COM_TibNew.get(1) - COMTibia.get(1), 2) + pow(COM_TibNew.get(2) - COMTibia.get(2), 2));
	dExoShank = sqrt(pow(COM_TibNew.get(1) - COMShank_Exo.get(1), 2) + pow(COM_TibNew.get(2) - COMShank_Exo.get(2), 2));
	dTalus = sqrt(pow(COM_FootNew.get(1) - COMTalus.get(1), 2) + pow(COM_FootNew.get(2) - COMTalus.get(2), 2));
	dExoFoot = sqrt(pow(COM_FootNew.get(1) - COMTalus.get(1), 2) + pow(COM_FootNew.get(2) - COMTalus.get(2), 2));
	double IFootTotalx = IFootExo.get(0) + mFoot_Exo * pow(dExoFoot, 2) + ITalus.get(0) + mTalus * pow(dTalus, 2);
	double IShankTotalx = IShankExo.get(0) + mShank_Exo * pow(dExoShank, 2) + ITibia.get(0) + mTibia * pow(dTibia, 2);

	// Get the new inertia: y -axis	
	dTibia = sqrt(pow(COM_TibNew.get(0) - COMTibia.get(0), 2) + pow(COM_TibNew.get(2) - COMTibia.get(2), 2));
	dExoShank = sqrt(pow(COM_TibNew.get(0) - COMShank_Exo.get(0), 2) + pow(COM_TibNew.get(2) - COMShank_Exo.get(2), 2));
	dTalus = sqrt(pow(COM_FootNew.get(0) - COMTalus.get(0), 2) + pow(COM_FootNew.get(2) - COMTalus.get(2), 2));
	dExoFoot = sqrt(pow(COM_FootNew.get(0) - COMFoot_Exo.get(0), 2) + pow(COM_FootNew.get(2) - COMTalus.get(2), 2));
	double IFootTotaly = IFootExo.get(1) + mFoot_Exo * pow(dExoFoot, 2) + ITalus.get(1) + mTalus * pow(dTalus, 2);
	double IShankTotaly = IShankExo.get(1) + mShank_Exo * pow(dExoShank, 2) + ITibia.get(1) + mTibia * pow(dTibia, 2);

	// Get the new inertia: z -axis	
	dTibia = sqrt(pow(COM_TibNew.get(0) - COMTibia.get(0), 2) + pow(COM_TibNew.get(1) - COMTibia.get(1), 2));
	dExoShank = sqrt(pow(COM_TibNew.get(0) - COMShank_Exo.get(0), 2) + pow(COM_TibNew.get(1) - COMShank_Exo.get(1), 2));
	dTalus = sqrt(pow(COM_FootNew.get(0) - COMTalus.get(0), 2) + pow(COM_FootNew.get(1) - COMTalus.get(1), 2));
	dExoFoot = sqrt(pow(COM_FootNew.get(0) - COMFoot_Exo.get(0), 2) + pow(COM_FootNew.get(1) - COMTalus.get(1), 2));
	double IFootTotalz = IFootExo.get(2) + mFoot_Exo * pow(dExoFoot, 2) + ITalus.get(2) + mTalus * pow(dTalus, 2);
	double IShankTotalz = IShankExo.get(2) + mShank_Exo * pow(dExoShank, 2) + ITibia.get(2) + mTibia * pow(dTibia, 2);

	// resulting inertia
	Inertia ITibiaNew(IShankTotalx, IShankTotaly, IShankTotalz, 0, 0, 0);
	Inertia IFootNew(IFootTotalx, IFootTotaly, IFootTotalz, 0, 0, 0);


	// OpenSim model: initialize components
	/// Model
	model = new OpenSim::Model();

	/// Body specifications (Mass, Mass center, Inertia)
	pelvis = new OpenSim::Body("pelvis", 9.7143336091724, Vec3(-0.0682778, 0, 0), Inertia(0.0814928846050306, 0.0814928846050306, 0.0445427591530667, 0, 0, 0));
	femur_l = new OpenSim::Body("femur_l", 7.67231915023828, Vec3(0, -0.170467, 0), Inertia(0.111055472890139, 0.0291116288158616, 0.117110028170931, 0, 0, 0));
	femur_r = new OpenSim::Body("femur_r", 7.67231915023828, Vec3(0, -0.170467, 0), Inertia(0.111055472890139, 0.0291116288158616, 0.117110028170931, 0, 0, 0));
	tibia_l = new OpenSim::Body("tibia_l", mTib_tot, COM_TibNew, ITibiaNew);
	tibia_r = new OpenSim::Body("tibia_r", mTib_tot, COM_TibNew, ITibiaNew);
	talus_l = new OpenSim::Body("talus_l", mFoot_tot, COM_FootNew, IFootNew);
	talus_r = new OpenSim::Body("talus_r", mFoot_tot, COM_FootNew, IFootNew);
	calcn_l = new OpenSim::Body("calcn_l", 1.03107047732576, Vec3(0.0913924, 0.0274177, 0), Inertia(0.0009646, 0.0026870, 0.0028248, 0, 0, 0));
	calcn_r = new OpenSim::Body("calcn_r", 1.03107047732576, Vec3(0.0913924, 0.0274177, 0), Inertia(0.0009646, 0.0026870, 0.0028248, 0, 0, 0));
	toes_l = new OpenSim::Body("toes_l", 0.178663892311008, Vec3(0.0316218, 0.00548355, 0.0159937), Inertia(6.88967700910182e-005, 0.000137793540182036, 6.88967700910182e-005, 0, 0, 0));
	toes_r = new OpenSim::Body("toes_r", 0.178663892311008, Vec3(0.0316218, 0.00548355, -0.0159937), Inertia(6.88967700910182e-005, 0.000137793540182036, 6.88967700910182e-005, 0, 0, 0));
	torso = new OpenSim::Body("torso", 28.240278003209, Vec3(-0.0289722, 0.309037, 0), Inertia(1.14043571182129, 0.593400919285897, 1.14043571182129, 0, 0, 0));

	/// Joints
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
	ground_pelvis = new PlanarJoint("ground_pelvis", model->getGround(), Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0));
	hip_l = new CustomJoint("hip_l", *pelvis, Vec3(-0.0682778001711179, -0.0638353973311301, -0.0823306940058688), Vec3(0), *femur_l, Vec3(0), Vec3(0), st_hip_l);
	hip_r = new CustomJoint("hip_r", *pelvis, Vec3(-0.0682778001711179, -0.0638353973311301, 0.0823306940058688), Vec3(0), *femur_r, Vec3(0), Vec3(0), st_hip_r);
	knee_l = new CustomJoint("knee_l", *femur_l, Vec3(-0.00451221232146798, -0.396907245921447, 0), Vec3(0), *tibia_l, Vec3(0), Vec3(0), st_knee_l);
	knee_r = new CustomJoint("knee_r", *femur_r, Vec3(-0.00451221232146798, -0.396907245921447, 0), Vec3(0), *tibia_r, Vec3(0), Vec3(0), st_knee_r);
	ankle_l = new PinJoint("ankle_l", *tibia_l, Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_l, Vec3(0), Vec3(0));
	ankle_r = new PinJoint("ankle_r", *tibia_r, Vec3(0, -0.415694825374905, 0), Vec3(0), *talus_r, Vec3(0), Vec3(0));
	subtalar_l = new WeldJoint("subtalar_l", *talus_l, Vec3(-0.0445720919117321, -0.0383391276542374, -0.00723828107321956), Vec3(0), *calcn_l, Vec3(0), Vec3(0));
	subtalar_r = new WeldJoint("subtalar_r", *talus_r, Vec3(-0.0445720919117321, -0.0383391276542374, 0.00723828107321956), Vec3(0), *calcn_r, Vec3(0), Vec3(0));
	mtp_l = new WeldJoint("mtp_l", *calcn_l, Vec3(0.163409678774199, -0.00182784875586352, -0.000987038328166303), Vec3(0), *toes_l, Vec3(0), Vec3(0));
	mtp_r = new WeldJoint("mtp_r", *calcn_r, Vec3(0.163409678774199, -0.00182784875586352, 0.000987038328166303), Vec3(0), *toes_r, Vec3(0), Vec3(0));
	back = new PinJoint("back", *pelvis, Vec3(-0.0972499926058214, 0.0787077894476112, 0), Vec3(0), *torso, Vec3(0), Vec3(0));
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
	double radiusSphere_heel = 0.035;
	double radiussphere_front = 0.015;
	double stiffness_heel = 3067776;
	double stiffness_front = 3067776;
	double dissipation = 2.0;
	double staticFriction = 0.8;
	double dynamicFriction = 0.8;
	double viscousFriction = 0.5;
	double transitionVelocity = 0.2;
	Vec3 locSphere_heel_l = Vec3(0.031307527581931796, 0.010435842527310599, 0);
	Vec3 locsphere_front_l = Vec3(0.1774093229642802, -0.015653763790965898, -0.005217921263655299);
	Vec3 locSphere_heel_r = Vec3(0.031307527581931796, 0.010435842527310599, 0);
	Vec3 locsphere_front_r = Vec3(0.1774093229642802, -0.015653763790965898, 0.005217921263655299);
	Vec3 normal = Vec3(0, 1, 0);
	double offset = 0;
	/// Right foot contact shere specifications
	HC_heel_r = new HuntCrossleyForce_smooth("sphere_heel_r", "calcn_r", locSphere_heel_r, radiusSphere_heel,
		stiffness_heel, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
	HC_front_r = new HuntCrossleyForce_smooth("sphere_front_r", "calcn_r", locsphere_front_r, radiussphere_front,
		stiffness_front, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
	/// Add right foot contact spheres to model
	model->addComponent(HC_heel_r);
	HC_heel_r->connectSocket_body_sphere(*calcn_r);
	model->addComponent(HC_front_r);
	HC_front_r->connectSocket_body_sphere(*calcn_r);
	/// Left foot contact shere specifications
	HC_heel_l = new HuntCrossleyForce_smooth("sphere_heel_l", "calcn_l", locSphere_heel_l, radiusSphere_heel,
		stiffness_heel, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
	HC_front_l = new HuntCrossleyForce_smooth("sphere_front_l", "calcn_l", locsphere_front_l, radiussphere_front,
		stiffness_front, dissipation, staticFriction, dynamicFriction, viscousFriction, transitionVelocity, normal, offset);
	/// Add left foot contact spheres to model
	model->addComponent(HC_heel_l);
	HC_heel_l->connectSocket_body_sphere(*calcn_l);
	model->addComponent(HC_front_l);
	HC_front_l->connectSocket_body_sphere(*calcn_l);

	// Initialize system and state.
	SimTK::State* state;
	state = new State(model->initSystem());
    

    return 0;
}
