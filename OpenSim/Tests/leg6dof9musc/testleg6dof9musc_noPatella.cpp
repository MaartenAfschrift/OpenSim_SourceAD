#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Simulation/Model/ConditionalPathPoint.h>
#include <OpenSim/Simulation/Model/MovingPathPoint.h>
#include <adolc.h>

//#include <OpenSim/OpenSim.h>

//#define VISUALIZE

using namespace SimTK;
using namespace OpenSim;


int main() {
	Model* model = new Model();
	model->setName("leg6dof9musc_noPatella");

//#ifdef VISUALIZE
//	model->setUseVisualizer(true);
//#endif

	// Bodies
	/// Pelvis
	OpenSim::Body* pelvis = new OpenSim::Body("pelvis",
		10.75379227, Vec3(-0.0739512, 0, 0), Inertia(0.1027003, 0.08701554, 0.05784385, 0, 0, 0));
	/// Femur
	OpenSim::Body* femur = new OpenSim::Body("femur",
		8.54926632, Vec3(0, -0.199543, 0), Inertia(0.16956431, 0.0444489, 0.17880867, 0, 0, 0));
	/// Tibia
	OpenSim::Body* tibia = new OpenSim::Body("tibia",
		4.67404243, Vec3(0, -0.2098, 0), Inertia(0.08023514, 0.00811903, 0.08134951, 0, 0, 0));
	/// Talus
	OpenSim::Body* talus = new OpenSim::Body("talus",
		0.12606858, Vec3(0, 0, 0), Inertia(0.00202166, 0.00202166, 0.00202166, 0, 0, 0));
	/// Calcaneus
	OpenSim::Body* calcn = new OpenSim::Body("calcn",
		1.07224816, Vec3(0.102693, 0.0308079, 0), Inertia(0.00090462, 0.00361848, 0.00361848, 0, 0, 0));
	/// Toes
	OpenSim::Body* toes = new OpenSim::Body("toes",
		0.18614228, Vec3(0.0359425, 0.00616157, -0.0184847), Inertia(0, 0, 0.00090462, 0, 0, 0));

	// Joints
	/// Ground-Pelvis
	//SpatialTransform st_ground_pelvis;
	//osim_double_adouble rotation1constantvalue_ground_pelvis = 0.0;
	//st_ground_pelvis[0].setFunction(new Constant(rotation1constantvalue_ground_pelvis));
	//osim_double_adouble rotation2constantvalue_ground_pelvis = -0.01391361;
	//st_ground_pelvis[1].setFunction(new Constant(rotation2constantvalue_ground_pelvis));
	//st_ground_pelvis[2].setCoordinateNames(OpenSim::Array<std::string>("pelvis_tilt", 1, 1)); // to be confirmed
	//st_ground_pelvis[2].setFunction(new LinearFunction());
	//st_ground_pelvis[3].setFunction(new LinearFunction());
	//st_ground_pelvis[4].setFunction(new LinearFunction());
	//osim_double_adouble translation3constantvalue_ground_pelvis = 0;
	//st_ground_pelvis[5].setFunction(new Constant(translation3constantvalue_ground_pelvis));

	////std::cout << st_ground_pelvis[0].hasFunction() << std::endl;

	//CustomJoint* ground_pelvis = new CustomJoint("ground_pelvis",
	//	// Parent body, location in parent, orientation in parent.
	//	model->getGround(), Vec3(0.593075, 0, 0.017089), Vec3(0),
	//	// Child body, location in child, orientation in child.
	//	*pelvis, Vec3(0), Vec3(0),
	//	// spatialTransform
	//	st_ground_pelvis);

	///// Hip
	//SpatialTransform st_hip;
	//osim_double_adouble rotation1constantvalue_hip = -0.01555646;
	//st_hip[0].setFunction(new Constant(rotation1constantvalue_hip));
	//osim_double_adouble rotation2constantvalue_hip = -0.07460014;
	//st_hip[1].setFunction(new Constant(rotation2constantvalue_hip));
	//st_hip[2].setCoordinateNames(OpenSim::Array<std::string>("hip_flexion", 1, 1));
	//st_hip[2].setFunction(new LinearFunction());
	//osim_double_adouble translationallconstantvalue_hip = 0;
	//st_hip[3].setFunction(new Constant(translationallconstantvalue_hip));
	//st_hip[4].setFunction(new Constant(translationallconstantvalue_hip));
	//st_hip[5].setFunction(new Constant(translationallconstantvalue_hip));

	//CustomJoint* hip = new CustomJoint("hip",
	//	*pelvis, Vec3(-0.0739512, -0.0691397, 0.0873398), Vec3(0),
	//	*femur, Vec3(0), Vec3(0), st_hip);

	///// Knee
	//SpatialTransform st_knee;
	//osim_double_adouble rotation1constantvalue_knee = 0;
	//st_knee[0].setFunction(new Constant(rotation1constantvalue_knee));
	//osim_double_adouble rotation2constantvalue_knee = 0;
	//st_knee[1].setFunction(new Constant(rotation2constantvalue_knee));
	//st_knee[2].setCoordinateNames(OpenSim::Array<std::string>("knee_angle", 1, 1));
	//st_knee[2].setFunction(new LinearFunction());
	//int npx = 12;
	//osim_double_adouble angX[] = { -2.0944, -1.74533, -1.39626, -1.0472, -0.698132, -0.349066, -0.174533, 0.197344, 0.337395, 0.490178, 1.52146, 2.0944 };
	//osim_double_adouble kneeX[] = { -0.0037561, 0.00210107, 0.00482424, 0.00481249, 0.00248841, -0.00117378, -0.00363871, -0.00613534, -0.00637949, -0.00654265, -0.00637949, -0.00616234 };
	//SimmSpline tx(npx, angX, kneeX);
	//st_knee[3].setCoordinateNames(OpenSim::Array<std::string>("knee_angle", 1, 1));
	//st_knee[3].setFunction(tx);
	//int npy = 7;
	//osim_double_adouble angY[] = { -2.0944, -1.22173, -0.523599, -0.349066, -0.174533, 0.159149, 2.0944 };
	//osim_double_adouble kneeY[] = { -0.496039, -0.479137, -0.468338, -0.466695, -0.465521, -0.463953, -0.464817 };
	//SimmSpline ty(npy, angY, kneeY);
	//st_knee[4].setCoordinateNames(OpenSim::Array<std::string>("knee_angle", 1, 1));
	//st_knee[4].setFunction(ty);
	//osim_double_adouble translation3constantvalue_knee = 0;
	//st_knee[5].setFunction(new Constant(translation3constantvalue_knee));

	//CustomJoint* knee = new CustomJoint("knee",
	//	*femur, Vec3(0), Vec3(0),
	//	*tibia, Vec3(0), Vec3(0), st_knee);

	///// ankle
	//SpatialTransform st_ankle;
	//st_ankle[0].setAxis(Vec3(-0.10501355, -0.17402245, 0.97912632));
	//st_ankle[0].setCoordinateNames(OpenSim::Array<std::string>("ankle_angle", 1, 1));
	//st_ankle[0].setFunction(new LinearFunction());
	//osim_double_adouble rotation2constantvalue_ankle = 0;
	//st_ankle[1].setFunction(new Constant(rotation2constantvalue_ankle));
	//osim_double_adouble rotation3constantvalue_ankle = 0;
	//st_ankle[2].setAxis(Vec3(0.97912632, -0, 0.10501355));
	//st_ankle[2].setFunction(new Constant(rotation3constantvalue_ankle));
	//osim_double_adouble translationallconstantvalue_ankle = 0;
	//st_ankle[3].setFunction(new Constant(translationallconstantvalue_ankle));
	//st_ankle[4].setFunction(new Constant(translationallconstantvalue_ankle));
	//st_ankle[5].setFunction(new Constant(translationallconstantvalue_ankle));

	//CustomJoint* ankle = new CustomJoint("ankle",
	//	*tibia, Vec3(0, -0.483203, 0), Vec3(0),
	//	*talus, Vec3(0), Vec3(0), st_ankle);
	
	WeldJoint* ground_pelvis = new WeldJoint("ground_pelvis", model->getGround(), Vec3(0.593075, 0, 0.017089), Vec3(0), *pelvis, Vec3(0, 1, 0), Vec3(0));
	//PinJoint* ground_pelvis = new PinJoint("ground_pelvis", model->getGround(), Vec3(0.593075, 0, 0.017089), Vec3(0), *pelvis, Vec3(0, 1, 0), Vec3(0));
	PinJoint* hip = new PinJoint("hip",	*pelvis, Vec3(-0.0739512, -0.0691397, 0.0873398), Vec3(0), *femur, Vec3(0), Vec3(0));
	//PinJoint* knee = new PinJoint("knee", *femur, Vec3(0), Vec3(0), *tibia, Vec3(0), Vec3(0));
	PinJoint* knee = new PinJoint("knee", *femur, Vec3(0, -0.4, 0), Vec3(0), *tibia, Vec3(0), Vec3(0));
	PinJoint* ankle = new PinJoint("ankle", *tibia, Vec3(0, -0.483203, 0), Vec3(0), *talus, Vec3(0), Vec3(0));

	/// subtalar
	WeldJoint* subtalar = new WeldJoint("subtalar",
		*talus, Vec3(-0.0620508, -0.0531864, 0.0101307), Vec3(0),
		*calcn, Vec3(0), Vec3(0));

	/// mtp
	WeldJoint* mtp = new WeldJoint("mtp",
		*calcn, Vec3(0.18382, -0.00205386, 0.00102692), Vec3(0),
		*toes, Vec3(0), Vec3(0));

	// Add components to the model.
	/// bodies and joints
	model->addBody(pelvis);		model->addJoint(ground_pelvis);
	model->addBody(femur);		model->addJoint(hip);
	model->addBody(tibia);		model->addJoint(knee);
	model->addBody(talus);		model->addJoint(ankle);
	model->addBody(calcn);		model->addJoint(subtalar);
	model->addBody(toes);		model->addJoint(mtp);

	//// Get coordinates
	//const CoordinateSet& coordSet = model->getCoordinateSet();
	//const Coordinate& coord_hip_flexion = coordSet.get("hip_flexion");
	//const Coordinate& coord_knee_angle = coordSet.get("knee_angle");
	//const Coordinate& coord_ankle_angle = coordSet.get("ankle_angle");

	const Coordinate& coord_hip_flexion = hip->getCoordinate();
	const Coordinate& coord_knee_angle = knee->getCoordinate();
	const Coordinate& coord_ankle_angle = ankle->getCoordinate();


	//// Muscles
	/// biceps femoris long head
	PathActuator* bifemlh = new PathActuator();
	bifemlh->addNewPathPoint("bifemlh_r-P1", *pelvis, Vec3(-0.131752, -0.107287, 0.0726333));
	bifemlh->addNewPathPoint("bifemlh_r-P2", *tibia, Vec3(-0.0338242, -0.0404542, 0.0330713));
	bifemlh->addNewPathPoint("bifemlh_r-P3", *tibia, Vec3(-0.0262952, -0.0632659, 0.0385439));

	/// biceps femoris short head
	PathActuator* bifemsh = new PathActuator();
	bifemsh->addNewPathPoint("bifemsh_r-P1", *femur, Vec3(0.0058689, -0.247785, 0.0274665));
	bifemsh->addNewPathPoint("bifemsh_r-P2", *tibia, Vec3(-0.0338242, -0.0404542, 0.0330713));
	bifemsh->addNewPathPoint("bifemsh_r-P3", *tibia, Vec3(-0.0262952, -0.0632659, 0.0385439));

	/// gluteus maximus 2
	PathActuator* glut_max2 = new PathActuator();
	glut_max2->addNewPathPoint("glut_max2_r-P1", *pelvis, Vec3(-0.141103, 0.0184094, 0.058889));
	glut_max2->addNewPathPoint("glut_max2_r-P2", *pelvis, Vec3(-0.143928, -0.0543913, 0.0956031));
	glut_max2->addNewPathPoint("glut_max2_r-P3", *femur, Vec3(-0.050003, -0.0622103, 0.0343917));
	glut_max2->addNewPathPoint("glut_max2_r-P3", *femur, Vec3(-0.018311, -0.119256, 0.0491814));

	/// psoas
	PathActuator* psoas = new PathActuator();
	psoas->addNewPathPoint("psoas_r-P1", *pelvis, Vec3(-0.0676753, 0.0927789, 0.030229));
	psoas->addNewPathPoint("psoas_r-P2", *pelvis, Vec3(-0.0248945, -0.0596212, 0.0793903));
	ConditionalPathPoint* psoas_r_P3 = new ConditionalPathPoint();
	psoas_r_P3->setName("psoas_r-P3");
	psoas_r_P3->setCoordinate(coord_hip_flexion);
	psoas_r_P3->setBody(*pelvis);
	psoas_r_P3->setLocation(Vec3(-0.0301244, -0.0842019, 0.0853524));
	psoas_r_P3->setRangeMin(-1.5708);
	psoas_r_P3->setRangeMax(0.785398);
	psoas->updGeometryPath().updPathPointSet().adoptAndAppend(psoas_r_P3);
	psoas->addNewPathPoint("psoas_r-P4", *femur, Vec3(0.00187804, -0.0595106, 0.00446036));
	psoas->addNewPathPoint("psoas_r-P5", *femur, Vec3(-0.0220671, -0.0700747, 0.0122073));

	/// rectus femoris
	PathActuator* rect_fem = new PathActuator();
	rect_fem->addNewPathPoint("rect_fem_r-P1", *pelvis, Vec3(-0.0308566, -0.0325302, 0.101251));
	ConditionalPathPoint* rect_fem_r_P2 = new ConditionalPathPoint();
	rect_fem_r_P2->setName("rect_fem_r-P2");
	rect_fem_r_P2->setCoordinate(coord_knee_angle);
	rect_fem_r_P2->setBody(*femur);
	rect_fem_r_P2->setLocation(Vec3(0.0392043, -0.473033, 0.00223019));
	rect_fem_r_P2->setRangeMin(-2.61799);
	rect_fem_r_P2->setRangeMax(-1.45997);
	rect_fem->updGeometryPath().updPathPointSet().adoptAndAppend(rect_fem_r_P2);
	MovingPathPoint* rect_fem_r_P3 = new MovingPathPoint(); // How to set location in body? See xml
	rect_fem_r_P3->setName("rect_fem_r-P3");
	rect_fem_r_P3->setBody(*tibia);
	int np_rfX = 17;
	osim_double_adouble rfX_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
	osim_double_adouble rfX_y[] = { 0.0175082, 0.0202202, 0.0309116, 0.0333257, 0.0345676, 0.0410942, 0.0474297, 0.0506691, 0.0543783, 0.0600407, 0.0693987, 0.0694092, 0.0694196, 0.0711413, 0.075283, 0.0823732, 0.0644437 };
	SimmSpline SS_rf_x(np_rfX, rfX_x, rfX_y);
	rect_fem_r_P3->set_x_location(SS_rf_x);
	rect_fem_r_P3->setXCoordinate(coord_knee_angle);
	int np_rfY = 17;
	osim_double_adouble rfY_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
	osim_double_adouble rfY_y[] = { 0.0263083, 0.0267012, 0.0282215, 0.0284073, 0.0284468, 0.0280015, 0.0272362, 0.026795, 0.0263174, 0.025581, 0.0235803, 0.0235774, 0.0235745, 0.0230617, 0.0215479, 0.0179296, -0.0757138 };
	SimmSpline SS_rf_y(np_rfY, rfY_x, rfY_y);
	rect_fem_r_P3->set_y_location(SS_rf_y);
	rect_fem_r_P3->setYCoordinate(coord_knee_angle);
	int np_rfZ = 2;
	osim_double_adouble rfZ_x[] = { -2.0944, 0.1745 };
	osim_double_adouble rfZ_y[] = { 0.00157322, 0.00157322 };
	SimmSpline SS_rf_z(np_rfZ, rfZ_x, rfZ_y);
	rect_fem_r_P3->set_z_location(SS_rf_z);
	rect_fem_r_P3->setZCoordinate(coord_knee_angle);
	rect_fem->updGeometryPath().updPathPointSet().adoptAndAppend(rect_fem_r_P3);

	/// vastus intermedius
	PathActuator* vast_int = new PathActuator();
	vast_int->addNewPathPoint("vas_int_r-P1", *femur, Vec3(0.0340396, -0.225835, 0.0363872));
	vast_int->addNewPathPoint("vas_int_r-P2", *femur, Vec3(0.0393216, -0.244616, 0.0334527));
	ConditionalPathPoint* vas_int_r_P3 = new ConditionalPathPoint();
	vas_int_r_P3->setName("vas_int_r-P3");
	vas_int_r_P3->setCoordinate(coord_knee_angle);
	vas_int_r_P3->setBody(*femur);
	vas_int_r_P3->setLocation(Vec3(0.0402607, -0.473033, 0.00645579));
	vas_int_r_P3->setRangeMin(-2.61799);
	vas_int_r_P3->setRangeMax(-1.42);
	vast_int->updGeometryPath().updPathPointSet().adoptAndAppend(vas_int_r_P3);
	MovingPathPoint* vas_int_r_P4 = new MovingPathPoint(); // How to set location in body? See xml
	vas_int_r_P4->setName("vas_int_r-P4");
	vas_int_r_P4->setBody(*tibia);
	int np_viX = 17;
	osim_double_adouble viX_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
	osim_double_adouble viX_y[] = { 0.00929694, 0.0120089, 0.022704, 0.0251187, 0.0263607, 0.0328932, 0.0392704, 0.0425567, 0.0463615, 0.0522856, 0.0623256, 0.0623371, 0.0623486, 0.0642454, 0.0688531, 0.0769044, 0.0729095 };
	SimmSpline SS_vi_x(np_viX, viX_x, viX_y);
	vas_int_r_P4->set_x_location(SS_vi_x);
	vas_int_r_P4->setXCoordinate(coord_knee_angle);
	int np_viY = 17;
	osim_double_adouble viY_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472, -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453, 0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
	osim_double_adouble viY_y[] = { 0.0287664, 0.0291593, 0.0306917, 0.0308797, 0.0309195, 0.0304938, 0.0298616, 0.0295622, 0.0293503, 0.0292314, 0.0284217, 0.0284203, 0.0284189, 0.0281523, 0.0272157, 0.0245296, -0.0770547 };
	SimmSpline SS_vi_y(np_viY, viY_x, viY_y);
	vas_int_r_P4->set_y_location(SS_vi_y);
	vas_int_r_P4->setYCoordinate(coord_knee_angle);
	int np_viZ = 2;
	osim_double_adouble viZ_x[] = { -2.0944, 2.0944 };
	osim_double_adouble viZ_y[] = { 0.00202271, 0.00202271 };
	SimmSpline SS_vi_z(np_viZ, viZ_x, viZ_y);
	vas_int_r_P4->set_z_location(SS_vi_z);
	vas_int_r_P4->setZCoordinate(coord_knee_angle);
	vast_int->updGeometryPath().updPathPointSet().adoptAndAppend(vas_int_r_P4);

	/// gastrocnemius medialis
	PathActuator* med_gas = new PathActuator();
	med_gas->addNewPathPoint("med_gas_r-P1", *femur, Vec3(-0.0223018, -0.461178, -0.0275838));
	ConditionalPathPoint* med_gas_r_P2 = new ConditionalPathPoint();
	med_gas_r_P2->setName("med_gas_r-P2");
	med_gas_r_P2->setCoordinate(coord_knee_angle);
	med_gas_r_P2->setBody(*femur);
	med_gas_r_P2->setLocation(Vec3(-0.0352134, -0.472094, -0.0302835));
	med_gas_r_P2->setRangeMin(-0.785398);
	med_gas_r_P2->setRangeMax(0.174533);
	med_gas->updGeometryPath().updPathPointSet().adoptAndAppend(med_gas_r_P2);
	med_gas->addNewPathPoint("med_gas_r-P3", *calcn, Vec3(0, 0.0318348, -0.00544272));

	/// soleus
	PathActuator* soleus = new PathActuator();
	soleus->addNewPathPoint("soleus_r-P1", *tibia, Vec3(-0.00269695, -0.172268, 0.00797847));
	soleus->addNewPathPoint("soleus_r-P2", *calcn, Vec3(0, 0.0318348, -0.00544272));

	/// tibialis anterior
	PathActuator* tib_ant = new PathActuator();
	tib_ant->addNewPathPoint("tib_ant_r-P1", *tibia, Vec3(0.0201147, -0.182493, 0.0129229));
	tib_ant->addNewPathPoint("tib_ant_r-P2", *tibia, Vec3(0.0369707, -0.443985, -0.01989));
	tib_ant->addNewPathPoint("tib_ant_r-P3", *calcn, Vec3(0.11974, 0.0182793, -0.0313213));

	// add components to model
	/// muscles
	model->addForce(bifemlh);
	model->addForce(bifemsh);
	model->addForce(glut_max2);
	model->addForce(psoas);
	model->addForce(rect_fem);
	model->addForce(vast_int);
	model->addForce(med_gas);
	model->addForce(soleus);
	model->addForce(tib_ant);

	//State& state = model->initSystem();
	State* state = new State(model->initSystem());
	MomentArmSolver* maSolver = new MomentArmSolver(*model);
	//MomentArmSolver maSolver(*model);

	// Independent variables
	Vector Qs(3);
	Vector Us(3);
    Vector Ua(3);
	adouble* pert = new adouble[1];

	// Set independent variable values
	double* xp = new double[10];
	xp[0] = 1.2;
	xp[1] = 1.9;
	xp[2] = 1.1;
	xp[3] = 1.7;
	xp[4] = 1.5;
	xp[5] = 1.3;
	xp[6] = 2.8;
    xp[7] = 1.5;
    xp[8] = 1.9;
    xp[9] = 1.4;


	trace_on(1, 1);

	Qs[0] <<= xp[0];
	Us[0] <<= xp[1];
	Qs[1] <<= xp[2];
	Us[1] <<= xp[3];
	Qs[2] <<= xp[4];
	Us[2] <<= xp[5];
	pert[0] <<= xp[6];
    Ua[0] <<= xp[7];
    Ua[1] <<= xp[8];
    Ua[2] <<= xp[9];

	state->setQ(Qs);
	state->setU(Us);

    model->realizeVelocity(*state);

	// appliedMobilityForces
	Vector appliedMobilityForces(3);
	appliedMobilityForces.setToZero();

	// appliedBodyForces
	Vector_<SpatialVec> appliedBodyForces;
	appliedBodyForces.resize(7);
	appliedBodyForces.setToZero();
	/// Gravity
	Vec3 gravity(0);
	gravity[1] = -9.81;
	/// Perturbation
	Vec3 pertVec = Vec3(pert[0] * gravity[1], 0, 0);
	/// Weight
	Vec3 weightpelvis = pelvis->getMass()*gravity;
	Vec3 weightfemur = femur->getMass()*gravity;
	Vec3 weighttibia = tibia->getMass()*gravity;
	Vec3 weighttalus = talus->getMass()*gravity;
	Vec3 weightcalcn = calcn->getMass()*gravity;
	Vec3 weighttoes = toes->getMass()*gravity;

	/// Weight + Perturbation
	Vec3 weight_pert_pelvis = weightpelvis + pelvis->getMass()*pertVec;
	Vec3 weight_pert_femur = weightfemur + femur->getMass()*pertVec;
	Vec3 weight_pert_tibia = weighttibia + tibia->getMass()*pertVec;
	Vec3 weight_pert_talus = weighttalus + talus->getMass()*pertVec;
	Vec3 weight_pert_calcn = weightcalcn + calcn->getMass()*pertVec;
	Vec3 weight_pert_toes = weighttoes + toes->getMass()*pertVec;

	/// Add to model
	model->getMatterSubsystem().addInStationForce(*state, pelvis->getMobilizedBodyIndex(), pelvis->getMassCenter(), weight_pert_pelvis, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, femur->getMobilizedBodyIndex(), femur->getMassCenter(), weight_pert_femur, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, tibia->getMobilizedBodyIndex(), tibia->getMassCenter(), weight_pert_tibia, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, talus->getMobilizedBodyIndex(), talus->getMassCenter(), weight_pert_talus, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, calcn->getMobilizedBodyIndex(), calcn->getMassCenter(), weight_pert_calcn, appliedBodyForces);
	model->getMatterSubsystem().addInStationForce(*state, toes->getMobilizedBodyIndex(), toes->getMassCenter(), weight_pert_toes, appliedBodyForces);

	// knownUdot
	Vector knownUdot(3);
	knownUdot.setToZero();
	knownUdot[0] = Ua[0];
	knownUdot[1] = Ua[1];
	knownUdot[2] = Ua[2];

	// residualMobilityForces
	Vector residualMobilityForces(3);
	residualMobilityForces.setToZero();


	model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state, appliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);

	//std::cout << residualMobilityForces << std::endl;

    Vector muscletendonlengths(9);
    muscletendonlengths.setToZero();

	/// Muscle Tendon Lengths and Velocities
	muscletendonlengths[0] = bifemlh->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_bifemlh = bifemlh->getLengtheningSpeed(*state);

    muscletendonlengths[1] = bifemsh->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_bifemsh = bifemsh->getLengtheningSpeed(*state);

    muscletendonlengths[2] = glut_max2->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_glut_max2 = glut_max2->getLengtheningSpeed(*state);

    muscletendonlengths[3] = psoas->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_psoas = psoas->getLengtheningSpeed(*state);

    muscletendonlengths[4] = rect_fem->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_rect_fem = rect_fem->getLengtheningSpeed(*state);

    muscletendonlengths[5] = vast_int->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_vast_int = vast_int->getLengtheningSpeed(*state);

    muscletendonlengths[6] = med_gas->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_med_gas = med_gas->getLengtheningSpeed(*state);

    muscletendonlengths[7] = soleus->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_soleus = soleus->getLengtheningSpeed(*state);

    muscletendonlengths[8] = tib_ant->getLength(*state);
	//osim_double_adouble musclelengtheningspeed_tib_ant = tib_ant->getLengtheningSpeed(*state);

	//std::cout << "musclelength_bifemlh " << muscletendonlengths[0] << std::endl;
	////std::cout << "musclelengtheningspeed_bifemlh " << musclelengtheningspeed_bifemlh << std::endl;

	//std::cout << "musclelength_bifemsh " << muscletendonlengths[1] << std::endl;
	////std::cout << "musclelengtheningspeed_bifemsh " << musclelengtheningspeed_bifemsh << std::endl;

	//std::cout << "musclelength_glut_max2 " << muscletendonlengths[2] << std::endl;
	////std::cout << "musclelengtheningspeed_glut_max2 " << musclelengtheningspeed_glut_max2 << std::endl;

	//std::cout << "musclelength_psoas " << muscletendonlengths[3] << std::endl;
	////std::cout << "musclelengtheningspeed_psoas " << musclelengtheningspeed_psoas << std::endl;

	//std::cout << "musclelength_rect_fem " << muscletendonlengths[4] << std::endl;
	////std::cout << "musclelengtheningspeed_rect_fem " << musclelengtheningspeed_rect_fem << std::endl;

	//std::cout << "musclelength_vast_int " << muscletendonlengths[5] << std::endl;
	////std::cout << "musclelengtheningspeed_vast_int " << musclelengtheningspeed_vast_int << std::endl;

	//std::cout << "musclelength_med_gas " << muscletendonlengths[6] << std::endl;
	////std::cout << "musclelengtheningspeed_med_gas " << musclelengtheningspeed_med_gas << std::endl;

	//std::cout << "musclelength_soleus " << muscletendonlengths[7] << std::endl;
	////std::cout << "musclelengtheningspeed_soleus " << musclelengtheningspeed_soleus << std::endl;

	//std::cout << "musclelength_tib_ant " << muscletendonlengths[8] << std::endl;
	////std::cout << "musclelengtheningspeed_tib_ant " << musclelengtheningspeed_tib_ant << std::endl;

	// Moment Arms
    Vector momentarm_hip(9);
    momentarm_hip.setToZero();
    momentarm_hip[0] = maSolver->solve(*state, hip->getCoordinate(), bifemlh->getGeometryPath());
    momentarm_hip[1] = maSolver->solve(*state, hip->getCoordinate(), bifemsh->getGeometryPath());
    momentarm_hip[2] = maSolver->solve(*state, hip->getCoordinate(), glut_max2->getGeometryPath());
    momentarm_hip[3] = maSolver->solve(*state, hip->getCoordinate(), psoas->getGeometryPath());
    momentarm_hip[4] = maSolver->solve(*state, hip->getCoordinate(), rect_fem->getGeometryPath());
    momentarm_hip[5] = maSolver->solve(*state, hip->getCoordinate(), vast_int->getGeometryPath());
    momentarm_hip[6] = maSolver->solve(*state, hip->getCoordinate(), med_gas->getGeometryPath());
    momentarm_hip[7] = maSolver->solve(*state, hip->getCoordinate(), soleus->getGeometryPath());
    momentarm_hip[8] = maSolver->solve(*state, hip->getCoordinate(), tib_ant->getGeometryPath());

    Vector momentarm_knee(9);
    momentarm_knee.setToZero();
    momentarm_knee[0] = maSolver->solve(*state, knee->getCoordinate(), bifemlh->getGeometryPath());
    momentarm_knee[1] = maSolver->solve(*state, knee->getCoordinate(), bifemsh->getGeometryPath());
    momentarm_knee[2] = maSolver->solve(*state, knee->getCoordinate(), glut_max2->getGeometryPath());
    momentarm_knee[3] = maSolver->solve(*state, knee->getCoordinate(), psoas->getGeometryPath());
    momentarm_knee[4] = maSolver->solve(*state, knee->getCoordinate(), rect_fem->getGeometryPath());
    momentarm_knee[5] = maSolver->solve(*state, knee->getCoordinate(), vast_int->getGeometryPath());
    momentarm_knee[6] = maSolver->solve(*state, knee->getCoordinate(), med_gas->getGeometryPath());
    momentarm_knee[7] = maSolver->solve(*state, knee->getCoordinate(), soleus->getGeometryPath());
    momentarm_knee[8] = maSolver->solve(*state, knee->getCoordinate(), tib_ant->getGeometryPath());

    Vector momentarm_ankle(9);
    momentarm_ankle.setToZero();
    momentarm_ankle[0] = maSolver->solve(*state, ankle->getCoordinate(), bifemlh->getGeometryPath());
    momentarm_ankle[1] = maSolver->solve(*state, ankle->getCoordinate(), bifemsh->getGeometryPath());
    momentarm_ankle[2] = maSolver->solve(*state, ankle->getCoordinate(), glut_max2->getGeometryPath());
    momentarm_ankle[3] = maSolver->solve(*state, ankle->getCoordinate(), psoas->getGeometryPath());
    momentarm_ankle[4] = maSolver->solve(*state, ankle->getCoordinate(), rect_fem->getGeometryPath());
    momentarm_ankle[5] = maSolver->solve(*state, ankle->getCoordinate(), vast_int->getGeometryPath());
    momentarm_ankle[6] = maSolver->solve(*state, ankle->getCoordinate(), med_gas->getGeometryPath());
    momentarm_ankle[7] = maSolver->solve(*state, ankle->getCoordinate(), soleus->getGeometryPath());
    momentarm_ankle[8] = maSolver->solve(*state, ankle->getCoordinate(), tib_ant->getGeometryPath());


	/////// bifemlh
	////adouble momentarm_bifemlh_hip = maSolver->solve(*state, coord_hip_flexion, bifemlh->getGeometryPath());
	//std::cout << "momentarm_bifemlh_hip_flexion " << momentarm_hip[0] << std::endl;
	////adouble momentarm_bifemlh_knee = maSolver->solve(*state, coord_knee_angle, bifemlh->getGeometryPath());
	//std::cout << "momentarm_bifemlh_knee_angle " << momentarm_knee[0] << std::endl;
	////adouble momentarm_bifemlh_ankle = maSolver->solve(*state, coord_ankle_angle, bifemlh->getGeometryPath());
	//std::cout << "momentarm_bifemlh_ankle_angle " << momentarm_ankle[0] << std::endl;

	/////// bifemsh
	////adouble momentarm_bifemsh_hip = maSolver->solve(*state, coord_hip_flexion, bifemsh->getGeometryPath());
	//std::cout << "momentarm_bifemsh_hip_flexion " << momentarm_hip[1] << std::endl;
	////adouble momentarm_bifemsh_knee = maSolver->solve(*state, coord_knee_angle, bifemsh->getGeometryPath());
	//std::cout << "momentarm_bifemsh_knee_angle " << momentarm_knee[1] << std::endl;
	////adouble momentarm_bifemsh_ankle = maSolver->solve(*state, coord_ankle_angle, bifemsh->getGeometryPath());
	//std::cout << "momentarm_bifemsh_ankle_angle " << momentarm_ankle[1] << std::endl;

	/////// glut_max2
	////adouble momentarm_glut_max2_hip = maSolver->solve(*state, coord_hip_flexion, glut_max2->getGeometryPath());
	//std::cout << "momentarm_glut_max2_hip_flexion " << momentarm_hip[2] << std::endl;
	////adouble momentarm_glut_max2_knee = maSolver->solve(*state, coord_knee_angle, glut_max2->getGeometryPath());
	//std::cout << "momentarm_glut_max2_knee_angle " << momentarm_knee[2] << std::endl;
	////adouble momentarm_glut_max2_ankle = maSolver->solve(*state, coord_ankle_angle, glut_max2->getGeometryPath());
	//std::cout << "momentarm_glut_max2_ankle_angle " << momentarm_ankle[2] << std::endl;

	/////// psoas
	////adouble momentarm_psoas_hip = maSolver->solve(*state, coord_hip_flexion, psoas->getGeometryPath());
	//std::cout << "momentarm_psoas_hip_flexion " << momentarm_hip[3] << std::endl;
	////adouble momentarm_psoas_knee = maSolver->solve(*state, coord_knee_angle, psoas->getGeometryPath());
	//std::cout << "momentarm_psoas_knee_angle " << momentarm_knee[3] << std::endl;
	////adouble momentarm_psoas_ankle = maSolver->solve(*state, coord_ankle_angle, psoas->getGeometryPath());
	//std::cout << "momentarm_psoas_ankle_angle " << momentarm_ankle[3] << std::endl;

	/////// rect_fem
	////adouble momentarm_rect_fem_hip = maSolver->solve(*state, coord_hip_flexion, rect_fem->getGeometryPath());
	//std::cout << "momentarm_rect_fem_hip_flexion " << momentarm_hip[4] << std::endl;
	////adouble momentarm_rect_fem_knee = maSolver->solve(*state, coord_knee_angle, rect_fem->getGeometryPath());
	//std::cout << "momentarm_rect_fem_knee_angle " << momentarm_knee[4] << std::endl;
	////adouble momentarm_rect_fem_ankle = maSolver->solve(*state, coord_ankle_angle, rect_fem->getGeometryPath());
	//std::cout << "momentarm_rect_fem_ankle_angle " << momentarm_ankle[4] << std::endl;

	/////// vast_int
	////adouble momentarm_vast_int_hip = maSolver->solve(*state, coord_hip_flexion, vast_int->getGeometryPath());
	//std::cout << "momentarm_vast_int_hip_flexion " << momentarm_hip[5] << std::endl;
	////adouble momentarm_vast_int_knee = maSolver->solve(*state, coord_knee_angle, vast_int->getGeometryPath());
	//std::cout << "momentarm_vast_int_knee_angle " << momentarm_knee[5] << std::endl;
	////adouble momentarm_vast_int_ankle = maSolver->solve(*state, coord_ankle_angle, vast_int->getGeometryPath());
	//std::cout << "momentarm_vast_int_ankle_angle " << momentarm_ankle[5] << std::endl;

	/////// med_gas
	////adouble momentarm_med_gas_hip = maSolver->solve(*state, coord_hip_flexion, med_gas->getGeometryPath());
	//std::cout << "momentarm_med_gas_hip_flexion " << momentarm_hip[6] << std::endl;
	////adouble momentarm_med_gas_knee = maSolver->solve(*state, coord_knee_angle, med_gas->getGeometryPath());
	//std::cout << "momentarm_med_gas_knee_angle " << momentarm_knee[6] << std::endl;
	////adouble momentarm_med_gas_ankle = maSolver->solve(*state, coord_ankle_angle, med_gas->getGeometryPath());
	//std::cout << "momentarm_med_gas_ankle_angle " << momentarm_ankle[6] << std::endl;

	/////// soleus
	////adouble momentarm_soleus_hip = maSolver->solve(*state, coord_hip_flexion, soleus->getGeometryPath());
	//std::cout << "momentarm_soleus_hip_flexion " << momentarm_hip[7] << std::endl;
	////adouble momentarm_soleus_knee = maSolver->solve(*state, coord_knee_angle, soleus->getGeometryPath());
	//std::cout << "momentarm_soleus_knee_angle " << momentarm_knee[7] << std::endl;
	////adouble momentarm_soleus_ankle = maSolver->solve(*state, coord_ankle_angle, soleus->getGeometryPath());
	//std::cout << "momentarm_soleus_ankle_angle " << momentarm_ankle[7] << std::endl;

	/////// tib_ant
	////adouble momentarm_tib_ant_hip = maSolver->solve(*state, coord_hip_flexion, tib_ant->getGeometryPath());
	//std::cout << "momentarm_tib_ant_hip_flexion " << momentarm_hip[8] << std::endl;
	////adouble momentarm_tib_ant_knee = maSolver->solve(*state, coord_knee_angle, tib_ant->getGeometryPath());
	//std::cout << "momentarm_tib_ant_knee_angle " << momentarm_knee[8] << std::endl;
	////adouble momentarm_tib_ant_ankle = maSolver->solve(*state, coord_ankle_angle, tib_ant->getGeometryPath());
	//std::cout << "momentarm_tib_ant_ankle_angle " << momentarm_ankle[8] << std::endl;

    // dependent variables

    adouble y_residualMobilityForces[3];
    y_residualMobilityForces[0] = residualMobilityForces[0];
    y_residualMobilityForces[1] = residualMobilityForces[1];
    y_residualMobilityForces[2] = residualMobilityForces[2];

    double residualMobilityForces_f[3];
    for (int i = 0; i < 3; ++i) {
        y_residualMobilityForces[i] >>= residualMobilityForces_f[i];
    }


    adouble y_muscletendonlengths[9];
    y_muscletendonlengths[0] = muscletendonlengths[0];
    y_muscletendonlengths[1] = muscletendonlengths[1];
    y_muscletendonlengths[2] = muscletendonlengths[2];
    y_muscletendonlengths[3] = muscletendonlengths[3];
    y_muscletendonlengths[4] = muscletendonlengths[4];
    y_muscletendonlengths[5] = muscletendonlengths[5];
    y_muscletendonlengths[6] = muscletendonlengths[6];
    y_muscletendonlengths[7] = muscletendonlengths[7];
    y_muscletendonlengths[8] = muscletendonlengths[8];

    double muscletendonlengths_f[9];
    for (int i = 0; i < 9; ++i) {
        y_muscletendonlengths[i] >>= muscletendonlengths_f[i];
    }

    adouble y_momentarm_hip[9];
    y_momentarm_hip[0] = momentarm_hip[0];
    y_momentarm_hip[1] = momentarm_hip[1];
    y_momentarm_hip[2] = momentarm_hip[2];
    y_momentarm_hip[3] = momentarm_hip[3];
    y_momentarm_hip[4] = momentarm_hip[4];
    y_momentarm_hip[5] = momentarm_hip[5];
    y_momentarm_hip[6] = momentarm_hip[6];
    y_momentarm_hip[7] = momentarm_hip[7];
    y_momentarm_hip[8] = momentarm_hip[8];

    double momentarm_hip_f[9];
    for (int i = 0; i < 9; ++i) {
        y_momentarm_hip[i] >>= momentarm_hip_f[i];
    }

    adouble y_momentarm_knee[9];
    y_momentarm_knee[0] = momentarm_knee[0];
    y_momentarm_knee[1] = momentarm_knee[1];
    y_momentarm_knee[2] = momentarm_knee[2];
    y_momentarm_knee[3] = momentarm_knee[3];
    y_momentarm_knee[4] = momentarm_knee[4];
    y_momentarm_knee[5] = momentarm_knee[5];
    y_momentarm_knee[6] = momentarm_knee[6];
    y_momentarm_knee[7] = momentarm_knee[7];
    y_momentarm_knee[8] = momentarm_knee[8];

    double momentarm_knee_f[9];
    for (int i = 0; i < 9; ++i) {
        y_momentarm_knee[i] >>= momentarm_knee_f[i];
    }

    adouble y_momentarm_ankle[9];
    y_momentarm_ankle[0] = momentarm_ankle[0];
    y_momentarm_ankle[1] = momentarm_ankle[1];
    y_momentarm_ankle[2] = momentarm_ankle[2];
    y_momentarm_ankle[3] = momentarm_ankle[3];
    y_momentarm_ankle[4] = momentarm_ankle[4];
    y_momentarm_ankle[5] = momentarm_ankle[5];
    y_momentarm_ankle[6] = momentarm_ankle[6];
    y_momentarm_ankle[7] = momentarm_ankle[7];
    y_momentarm_ankle[8] = momentarm_ankle[8];

    double momentarm_ankle_f[9];
    for (int i = 0; i < 9; ++i) {
        y_momentarm_ankle[i] >>= momentarm_ankle_f[i];
    }
    
    trace_off();

    double nominal_res[39];;
    zos_forward(1, 39, 10, 0, xp, nominal_res); 
    printf("Zos forward \n %f %f %f \n %f %f %f %f %f %f %f %f %f \n %f %f %f %f %f %f %f %f %f \n %f %f %f %f %f %f %f %f %f \n %f %f %f %f %f %f %f %f %f \n \n",
        nominal_res[0], nominal_res[1], nominal_res[2],
        nominal_res[3], nominal_res[4], nominal_res[5], nominal_res[6], nominal_res[7], nominal_res[8], nominal_res[9], nominal_res[10], nominal_res[11],
        nominal_res[12], nominal_res[13], nominal_res[14], nominal_res[15], nominal_res[16], nominal_res[17], nominal_res[18], nominal_res[19], nominal_res[20], 
        nominal_res[21], nominal_res[22], nominal_res[23], nominal_res[24], nominal_res[25], nominal_res[26], nominal_res[27], nominal_res[28], nominal_res[29], 
        nominal_res[30], nominal_res[31], nominal_res[32], nominal_res[33], nominal_res[34], nominal_res[35], nominal_res[36], nominal_res[37], nominal_res[38]);

	return 0;
};