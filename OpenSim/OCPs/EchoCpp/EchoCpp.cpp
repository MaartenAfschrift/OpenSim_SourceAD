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
#include <recorder.hpp>

#include <iostream>
#include <iterator>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <fstream>
#include <chrono>

/*#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <opensim/simulation/model/marker.h>
#include <opensim/simulation/model/markerset.h>
#include <OpenSim/Simulation/model/ExpressionBasedBushingForce.h>*/

using namespace SimTK;
using namespace OpenSim;

//(2-spine)  input all q qdot qdotdots
constexpr int n_in = 3; //(two vectors of input arguments,q,qdots,qdotdots) + one vecvor of marker index
constexpr int n_out = 1; //(one vectors of output arguments, joint torques)  + one vecvor of marker position
						 /// number of elements in input/output vectors of function F
constexpr int ndofr = 229;       // degrees of freedom (including locked=all joints)
constexpr int NX = ndofr * 2;      // states (position and velocities of TRACKED joints)
constexpr int NU = ndofr;        // controls  (TRACKED)
constexpr int NQall = ndofr;    // residual torques + GRFs + MOMs. only for output?
								//Echo: add input GRF+GRM-COP , delete MoM
constexpr int nMkr = 27; //Echo: number of tracked markers
constexpr int nEBBEw = 102; //Echo: number of tracked markers, 17*6 for 6dof
constexpr int NR = NQall + nMkr * 3;      // number of results


template<typename T>
T value(const Recorder& e) { return e; }

template<>
double value(const Recorder& e) { return e.getValue(); }

/* Function F, using templated type T
F(x,u) -> (Tau)
*/

template<typename T>
int F_generic(const T** arg, T** res) {


	OpenSim::Model* model;
	SimTK::State* state;
	model = new OpenSim::Model();

	//OpenSim::MarkerSet* markerSet;

	//String EBBElist[17] = { "EBBE_L5-S1", "EBBE_L4-L5", "EBBE_L3-L4", "EBBE_L2-L3", "EBBE_L1-L2",
	//	"EBBE_T12-L1", "EBBE_T11-T12", "EBBE_T10-T11", "EBBE_T9-T10", "EBBE_T8-T9", "EBBE_T7-T8", "EBBE_T6-T7",
	//	"EBBE_T5-T6", "EBBE_T4-T5",  "EBBE_T3-T4",  "EBBE_T2-T3", "EBBE_T1-T2" };

	//String MkrNamesOSIM[nMkr] = { "T1L", "T1M", "T1R",
	//	"T3M","T3L","T3R","T5","T7M","T7L","T7R","T9","T11M","T11L","T11R", "T12",
	//	"L2M","L2L","L2R","L3","L4M","L4L","L4R",
	//	"SACR", "RPSI", "LPSI", "LASI", "RASI" };
	int* idx_states;
	int* idx_states2;



	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////

	OpenSim::Body* sacrum;
	OpenSim::Body* pelvis;
	OpenSim::Body* Abdomen;
	OpenSim::Body* lumbar5;
	OpenSim::Body* lumbar4;
	OpenSim::Body* lumbar3;
	OpenSim::Body* lumbar2;
	OpenSim::Body* lumbar1;
	OpenSim::Body* thoracic12;
	OpenSim::Body* thoracic11;
	OpenSim::Body* thoracic10;
	OpenSim::Body* thoracic9;
	OpenSim::Body* thoracic8;
	OpenSim::Body* thoracic7;
	OpenSim::Body* thoracic6;
	OpenSim::Body* thoracic5;
	OpenSim::Body* thoracic4;
	OpenSim::Body* thoracic3;
	OpenSim::Body* thoracic2;
	OpenSim::Body* thoracic1;
	OpenSim::Body* cerv7;
	OpenSim::Body* cerv6;
	OpenSim::Body* cerv5;
	OpenSim::Body* cerv4;
	OpenSim::Body* cerv3;
	OpenSim::Body* cerv2;
	OpenSim::Body* cerv1;
	OpenSim::Body* skull;
	OpenSim::Body* jaw;
	OpenSim::Body* rib12_R;
	OpenSim::Body* rib11_R;
	OpenSim::Body* rib10_R;
	OpenSim::Body* rib9_R;
	OpenSim::Body* rib8_R;
	OpenSim::Body* rib7_R;
	OpenSim::Body* rib6_R;
	OpenSim::Body* rib5_R;
	OpenSim::Body* rib4_R;
	OpenSim::Body* rib3_R;
	OpenSim::Body* rib2_R;
	OpenSim::Body* rib1_R;
	OpenSim::Body* rib12_L;
	OpenSim::Body* rib11_L;
	OpenSim::Body* rib10_L;
	OpenSim::Body* rib9_L;
	OpenSim::Body* rib8_L;
	OpenSim::Body* rib7_L;
	OpenSim::Body* rib6_L;
	OpenSim::Body* rib5_L;
	OpenSim::Body* rib4_L;
	OpenSim::Body* rib3_L;
	OpenSim::Body* rib2_L;
	OpenSim::Body* rib1_L;
	OpenSim::Body* sternum;
	OpenSim::Body* clavicle_R;
	OpenSim::Body* scapula_R;
	OpenSim::Body* humerus_R;
	OpenSim::Body* ulna_R;
	OpenSim::Body* radius_R;
	OpenSim::Body* hand_R;
	OpenSim::Body* humerus_L;
	OpenSim::Body* ulna_L;
	OpenSim::Body* radius_L;
	OpenSim::Body* hand_L;
	OpenSim::Body* clavicle_L;
	OpenSim::Body* scapula_L;
	OpenSim::Body* Abd_L_L1;
	OpenSim::Body* Abd_L_L2;
	OpenSim::Body* Abd_L_L3;
	OpenSim::Body* Abd_L_L4;
	OpenSim::Body* Abd_L_L5;
	OpenSim::Body* Abd_R_L1;
	OpenSim::Body* Abd_R_L2;
	OpenSim::Body* Abd_R_L3;
	OpenSim::Body* Abd_R_L4;
	OpenSim::Body* Abd_R_L5;
	//
	OpenSim::CustomJoint* gndsacrum;
	OpenSim::WeldJoint* sacrum_pelvis_weld;
	OpenSim::CustomJoint* Abdjnt;
	OpenSim::CustomJoint* L5_S1_IVDjnt;
	OpenSim::CustomJoint* L4_L5_IVDjnt;
	OpenSim::CustomJoint* L3_L4_IVDjnt;
	OpenSim::CustomJoint* L2_L3_IVDjnt;
	OpenSim::CustomJoint* L1_L2_IVDjnt;
	OpenSim::CustomJoint* T12_L1_IVDjnt;
	OpenSim::CustomJoint* T11_T12_IVDjnt;
	OpenSim::CustomJoint* T10_T11_IVDjnt;
	OpenSim::CustomJoint* T9_T10_IVDjnt;
	OpenSim::CustomJoint* T8_T9_IVDjnt;
	OpenSim::CustomJoint* T7_T8_IVDjnt;
	OpenSim::CustomJoint* T6_T7_IVDjnt;
	OpenSim::CustomJoint* T5_T6_IVDjnt;
	OpenSim::CustomJoint* T4_T5_IVDjnt;
	OpenSim::CustomJoint* T3_T4_IVDjnt;
	OpenSim::CustomJoint* T2_T3_IVDjnt;
	OpenSim::CustomJoint* T1_T2_IVDjnt;
	OpenSim::CustomJoint* auxt1jnt;
	OpenSim::CustomJoint* aux7jnt;
	OpenSim::CustomJoint* aux6jnt;
	OpenSim::CustomJoint* aux5jnt;
	OpenSim::CustomJoint* aux4jnt;
	OpenSim::CustomJoint* aux3jnt;
	OpenSim::CustomJoint* aux2jnt;
	OpenSim::CustomJoint* aux1jnt;
	OpenSim::WeldJoint* jawjnt_weld;
	OpenSim::CustomJoint* T12_r12R_CVjnt;
	OpenSim::CustomJoint* T11_r11R_CVjnt;
	OpenSim::CustomJoint* T10_r10R_CVjnt;
	OpenSim::CustomJoint* T9_r9R_CVjnt;
	OpenSim::CustomJoint* T8_r8R_CVjnt;
	OpenSim::CustomJoint* T7_r7R_CVjnt;
	OpenSim::CustomJoint* T6_r6R_CVjnt;
	OpenSim::CustomJoint* T5_r5R_CVjnt;
	OpenSim::CustomJoint* T4_r4R_CVjnt;
	OpenSim::CustomJoint* T3_r3R_CVjnt;
	OpenSim::CustomJoint* T2_r2R_CVjnt;
	OpenSim::CustomJoint* T1_r1R_CVjnt;
	OpenSim::CustomJoint* T12_r12L_CVjnt;
	OpenSim::CustomJoint* T11_r11L_CVjnt;
	OpenSim::CustomJoint* T10_r10L_CVjnt;
	OpenSim::CustomJoint* T9_r9L_CVjnt;
	OpenSim::CustomJoint* T8_r8L_CVjnt;
	OpenSim::CustomJoint* T7_r7L_CVjnt;
	OpenSim::CustomJoint* T6_r6L_CVjnt;
	OpenSim::CustomJoint* T5_r5L_CVjnt;
	OpenSim::CustomJoint* T4_r4L_CVjnt;
	OpenSim::CustomJoint* T3_r3L_CVjnt;
	OpenSim::CustomJoint* T2_r2L_CVjnt;
	OpenSim::CustomJoint* T1_r1L_CVjnt;
	OpenSim::CustomJoint* r1R_sterR_jnt;
	OpenSim::WeldJoint* sterR_clavR_jnt_weld;
	OpenSim::WeldJoint* clavR_scapR_jnt_weld;
	OpenSim::CustomJoint* shoulder_R;
	OpenSim::CustomJoint* elbow;
	OpenSim::CustomJoint* radioulnar;
	OpenSim::CustomJoint* radius_hand_r;
	OpenSim::CustomJoint* shoulder_L;
	OpenSim::CustomJoint* elbow_l;
	OpenSim::CustomJoint* radioulnar_l;
	OpenSim::CustomJoint* radius_hand_l;
	OpenSim::WeldJoint* sterL_clavL_jnt_weld;
	OpenSim::WeldJoint* clavL_scapL_jnt_weld;
	OpenSim::WeldJoint* Abd_L_L1_weld;
	OpenSim::WeldJoint* Abd_L_L2_weld;
	OpenSim::WeldJoint* Abd_L_L3_weld;
	OpenSim::WeldJoint* Abd_L_L4_weld;
	OpenSim::WeldJoint* Abd_L_L5_weld;
	OpenSim::WeldJoint* Abd_R_L1_weld;
	OpenSim::WeldJoint* Abd_R_L2_weld;
	OpenSim::WeldJoint* Abd_R_L3_weld;
	OpenSim::WeldJoint* Abd_R_L4_weld;
	OpenSim::WeldJoint* Abd_R_L5_weld;
	//
	//OpenSim::Marker* C7;
	//OpenSim::Marker* T1L;
	//OpenSim::Marker* T1M;
	//OpenSim::Marker* T1R;
	//OpenSim::Marker* T3M;
	//OpenSim::Marker* T3L;
	//OpenSim::Marker* T3R;
	//OpenSim::Marker* T5;
	//OpenSim::Marker* T7M;
	//OpenSim::Marker* T7L;
	//OpenSim::Marker* T7R;
	//OpenSim::Marker* T9;
	//OpenSim::Marker* T11M;
	//OpenSim::Marker* T11L;
	//OpenSim::Marker* T11R;
	//OpenSim::Marker* T12;
	//OpenSim::Marker* L2M;
	//OpenSim::Marker* L2L;
	//OpenSim::Marker* L2R;
	//OpenSim::Marker* L3;
	//OpenSim::Marker* L4M;
	//OpenSim::Marker* L4L;
	//OpenSim::Marker* SACR;
	//OpenSim::Marker* L4R;
	//OpenSim::Marker* RPSI;
	//OpenSim::Marker* LPSI;
	//OpenSim::Marker* LASI;
	//OpenSim::Marker* RASI;
	//OpenSim::Marker* STRN;
	//OpenSim::Marker* LFHD;
	//OpenSim::Marker* RFHD;
	//OpenSim::Marker* LBHD;
	//OpenSim::Marker* RBHD;
	//OpenSim::Marker* LSHO;
	//OpenSim::Marker* RSHO;
	//OpenSim::Marker* CLAV;
	//OpenSim::Marker* RBAK;
	//OpenSim::Marker* T3AA;
	//OpenSim::Marker* T7AA;
	//OpenSim::Marker* T11AA;
	//OpenSim::Marker* L2AA;
	//OpenSim::Marker* L4AA;
	//OpenSim::Marker* LELB;
	//OpenSim::Marker* LELM;
	//OpenSim::Marker* RELB;
	//OpenSim::Marker* RELM;
	//OpenSim::Marker* LWRA;
	//OpenSim::Marker* LWRB;
	//OpenSim::Marker* RWRA;
	//OpenSim::Marker* RWRB;
	//


	sacrum = new OpenSim::Body("sacrum", 5.8834, Vec3(0, 0, 0), Inertia(5.4973e-05, 5.4973e-05, 5.4973e-05, 0, 0, 0));
	model->addBody(sacrum);
	//
	pelvis = new OpenSim::Body("pelvis", 7.052e-05, Vec3(0, 0, 0), Inertia(4.0703e-05, 4.7838e-05, 4.7838e-05, 0, 0, 0));
	model->addBody(pelvis);
	//
	Abdomen = new OpenSim::Body("Abdomen", 7.052e-05, Vec3(0, 0, 0), Inertia(7.0183e-05, 7.0183e-05, 7.0183e-05, 0, 0, 0));
	model->addBody(Abdomen);
	//
	lumbar5 = new OpenSim::Body("lumbar5", 1.549, Vec3(0.011377, 0.022118, 0), Inertia(0.0060064, 0.0060064, 0.0028216, 0, 0, 0));
	model->addBody(lumbar5);
	//
	lumbar4 = new OpenSim::Body("lumbar4", 1.47, Vec3(0.014318, 0.017042, 0), Inertia(0.005536, 0.005536, 0.0026872, 0, 0, 0));
	model->addBody(lumbar4);
	//
	lumbar3 = new OpenSim::Body("lumbar3", 1.43, Vec3(0.022582, 0.012421, 0), Inertia(0.0052639, 0.0052639, 0.0028474, 0, 0, 0));
	model->addBody(lumbar3);
	//
	lumbar2 = new OpenSim::Body("lumbar2", 1.3933, Vec3(0.028633, 0.0062972, 0), Inertia(0.0053915, 0.0053915, 0.0030541, 0, 0, 0));
	model->addBody(lumbar2);
	//
	lumbar1 = new OpenSim::Body("lumbar1", 1.4194, Vec3(0.034169, 0.00098401, 0), Inertia(0.005506, 0.005506, 0.0033073, 0, 0, 0));
	model->addBody(lumbar1);
	//
	thoracic12 = new OpenSim::Body("thoracic12", 1.4008, Vec3(0.051236, -0.002035, 0), Inertia(0.0075685, 0.011921, 0.0043521, 0, 0, 0));
	model->addBody(thoracic12);
	//
	thoracic11 = new OpenSim::Body("thoracic11", 1.1318, Vec3(0.055462, -0.0013957, 0), Inertia(0.0075685, 0.011921, 0.0043521, 0, 0, 0));
	model->addBody(thoracic11);
	//
	thoracic10 = new OpenSim::Body("thoracic10", 1.0924, Vec3(0.059332, 0.0027093, 0), Inertia(0.0075685, 0.011921, 0.0043521, 0, 0, 0));
	model->addBody(thoracic10);
	//
	thoracic9 = new OpenSim::Body("thoracic9", 0.91788, Vec3(0.063176, 0.0045851, 0), Inertia(0.0075685, 0.011921, 0.0043521, 0, 0, 0));
	model->addBody(thoracic9);
	//
	thoracic8 = new OpenSim::Body("thoracic8", 0.8645, Vec3(0.058858, 0.010028, 0), Inertia(0.0075685, 0.011921, 0.0043521, 0, 0, 0));
	model->addBody(thoracic8);
	//
	thoracic7 = new OpenSim::Body("thoracic7", 0.89278, Vec3(0.054615, 0.016549, 0), Inertia(0.0075685, 0.011921, 0.0043521, 0, 0, 0));
	model->addBody(thoracic7);
	//
	thoracic6 = new OpenSim::Body("thoracic6", 0.80082, Vec3(0.048243, 0.020606, 0), Inertia(0.0069737, 0.010984, 0.00401, 0, 0, 0));
	model->addBody(thoracic6);
	//
	thoracic5 = new OpenSim::Body("thoracic5", 0.82367, Vec3(0.041788, 0.024367, 0), Inertia(0.0069737, 0.010984, 0.00401, 0, 0, 0));
	model->addBody(thoracic5);
	//
	thoracic4 = new OpenSim::Body("thoracic4", 0.73827, Vec3(0.034303, 0.025398, 0), Inertia(0.0069737, 0.010984, 0.00401, 0, 0, 0));
	model->addBody(thoracic4);
	//
	thoracic3 = new OpenSim::Body("thoracic3", 0.79786, Vec3(0.023039, 0.021706, 0), Inertia(0.0069737, 0.010984, 0.00401, 0, 0, 0));
	model->addBody(thoracic3);
	//
	thoracic2 = new OpenSim::Body("thoracic2", 0.66937, Vec3(0.017698, 0.019364, 0), Inertia(0.0069737, 0.010984, 0.00401, 0, 0, 0));
	model->addBody(thoracic2);
	//
	thoracic1 = new OpenSim::Body("thoracic1", 0.63007, Vec3(0.0041898, 0.013832, 0), Inertia(0.0069737, 0.010984, 0.00401, 0, 0, 0));
	model->addBody(thoracic1);
	//
	cerv7 = new OpenSim::Body("cerv7", 0.41189, Vec3(0.029409, 0.0017909, 0), Inertia(0.012531, 0.050125, 0.012531, 0, 0, 0));
	model->addBody(cerv7);
	//
	cerv6 = new OpenSim::Body("cerv6", 0.35806, Vec3(0.029409, 0.0017909, 0), Inertia(0.012531, 0.050125, 0.012531, 0, 0, 0));
	model->addBody(cerv6);
	//
	cerv5 = new OpenSim::Body("cerv5", 0.3026, Vec3(0.029409, 0.0017909, 0), Inertia(0.012531, 0.050125, 0.012531, 0, 0, 0));
	model->addBody(cerv5);
	//
	cerv4 = new OpenSim::Body("cerv4", 0.29852, Vec3(0.029409, 0.0017909, 0), Inertia(0.012531, 0.050125, 0.012531, 0, 0, 0));
	model->addBody(cerv4);
	//
	cerv3 = new OpenSim::Body("cerv3", 0.29607, Vec3(0.029409, 0.0017909, 0), Inertia(0.012531, 0.050125, 0.012531, 0, 0, 0));
	model->addBody(cerv3);
	//
	cerv2 = new OpenSim::Body("cerv2", 0.41434, Vec3(0.029409, 0.0017909, 0), Inertia(0.012531, 0.050125, 0.012531, 0, 0, 0));
	model->addBody(cerv2);
	//
	cerv1 = new OpenSim::Body("cerv1", 0.32952, Vec3(0.029409, 0.0017909, 0), Inertia(0.012531, 0.050125, 0.012531, 0, 0, 0));
	model->addBody(cerv1);
	//
	skull = new OpenSim::Body("skull", 2.6797, Vec3(0, 0.055997, 0), Inertia(0.01275, 0.0039192, 0.01375, 0, 0, 0));
	model->addBody(skull);
	//
	jaw = new OpenSim::Body("jaw", 0.14104, Vec3(0, 0.041418, 0), Inertia(0.019356, 0.0096778, 0.019356, 0, 0, 0));
	model->addBody(jaw);
	//
	rib12_R = new OpenSim::Body("rib12_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib12_R);
	//
	rib11_R = new OpenSim::Body("rib11_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib11_R);
	//
	rib10_R = new OpenSim::Body("rib10_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib10_R);
	//
	rib9_R = new OpenSim::Body("rib9_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib9_R);
	//
	rib8_R = new OpenSim::Body("rib8_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib8_R);
	//
	rib7_R = new OpenSim::Body("rib7_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib7_R);
	//
	rib6_R = new OpenSim::Body("rib6_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib6_R);
	//
	rib5_R = new OpenSim::Body("rib5_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib5_R);
	//
	rib4_R = new OpenSim::Body("rib4_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib4_R);
	//
	rib3_R = new OpenSim::Body("rib3_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib3_R);
	//
	rib2_R = new OpenSim::Body("rib2_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib2_R);
	//
	rib1_R = new OpenSim::Body("rib1_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib1_R);
	//
	rib12_L = new OpenSim::Body("rib12_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib12_L);
	//
	rib11_L = new OpenSim::Body("rib11_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib11_L);
	//
	rib10_L = new OpenSim::Body("rib10_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib10_L);
	//
	rib9_L = new OpenSim::Body("rib9_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib9_L);
	//
	rib8_L = new OpenSim::Body("rib8_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib8_L);
	//
	rib7_L = new OpenSim::Body("rib7_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib7_L);
	//
	rib6_L = new OpenSim::Body("rib6_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib6_L);
	//
	rib5_L = new OpenSim::Body("rib5_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib5_L);
	//
	rib4_L = new OpenSim::Body("rib4_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib4_L);
	//
	rib3_L = new OpenSim::Body("rib3_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib3_L);
	//
	rib2_L = new OpenSim::Body("rib2_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib2_L);
	//
	rib1_L = new OpenSim::Body("rib1_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.2656e-05, 6.2656e-05, 6.2656e-05, 0, 0, 0));
	model->addBody(rib1_L);
	//
	sternum = new OpenSim::Body("sternum", 7.052e-05, Vec3(0, 0, 0), Inertia(5.9411e-05, 5.9411e-05, 5.9411e-05, 0, 0, 0));
	model->addBody(sternum);
	//
	clavicle_R = new OpenSim::Body("clavicle_R", 7.052e-05, Vec3(0, 0, 0), Inertia(6.0604e-05, 6.0604e-05, 6.0604e-05, 0, 0, 0));
	model->addBody(clavicle_R);
	//
	scapula_R = new OpenSim::Body("scapula_R", 7.052e-05, Vec3(0, 0, 0), Inertia(5.5066e-05, 5.5066e-05, 5.5066e-05, 0, 0, 0));
	model->addBody(scapula_R);
	//
	humerus_R = new OpenSim::Body("humerus_R", 1.4906, Vec3(0, -0.10303, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(humerus_R);
	//
	ulna_R = new OpenSim::Body("ulna_R", 0.42841, Vec3(0, -0.10372, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(ulna_R);
	//
	radius_R = new OpenSim::Body("radius_R", 0.42841, Vec3(0, -0.10372, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(radius_R);
	//
	hand_R = new OpenSim::Body("hand_R", 0.32263, Vec3(0, -0.058602, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(hand_R);
	//
	humerus_L = new OpenSim::Body("humerus_L", 1.4906, Vec3(0, -0.10303, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(humerus_L);
	//
	ulna_L = new OpenSim::Body("ulna_L", 0.42841, Vec3(0, -0.10372, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(ulna_L);
	//
	radius_L = new OpenSim::Body("radius_L", 0.42841, Vec3(0, -0.10372, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(radius_L);
	//
	hand_L = new OpenSim::Body("hand_L", 0.32263, Vec3(0, -0.058602, 0), Inertia(5.2228e-05, 5.2228e-05, 5.2228e-05, 0, 0, 0));
	model->addBody(hand_L);
	//
	clavicle_L = new OpenSim::Body("clavicle_L", 7.052e-05, Vec3(0, 0, 0), Inertia(6.0604e-05, 6.0604e-05, 6.0604e-05, 0, 0, 0));
	model->addBody(clavicle_L);
	//
	scapula_L = new OpenSim::Body("scapula_L", 7.052e-05, Vec3(0, 0, 0), Inertia(5.5066e-05, 5.5066e-05, 5.5066e-05, 0, 0, 0));
	model->addBody(scapula_L);
	//
	Abd_L_L1 = new OpenSim::Body("Abd_L_L1", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_L_L1);
	//
	Abd_L_L2 = new OpenSim::Body("Abd_L_L2", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_L_L2);
	//
	Abd_L_L3 = new OpenSim::Body("Abd_L_L3", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_L_L3);
	//
	Abd_L_L4 = new OpenSim::Body("Abd_L_L4", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_L_L4);
	//
	Abd_L_L5 = new OpenSim::Body("Abd_L_L5", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_L_L5);
	//
	Abd_R_L1 = new OpenSim::Body("Abd_R_L1", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_R_L1);
	//
	Abd_R_L2 = new OpenSim::Body("Abd_R_L2", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_R_L2);
	//
	Abd_R_L3 = new OpenSim::Body("Abd_R_L3", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_R_L3);
	//
	Abd_R_L4 = new OpenSim::Body("Abd_R_L4", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_R_L4);
	//
	Abd_R_L5 = new OpenSim::Body("Abd_R_L5", 0.007052, Vec3(0, 0, 0), Inertia(0.0051677, 0.0050128, 0.0050128, 0, 0, 0));
	model->addBody(Abd_R_L5);
	//
	//
	SpatialTransform st_sacrum_gndsacrum;
	st_sacrum_gndsacrum[0].setCoordinateNames(OpenSim::Array<std::string>("sacrum_tilt", 1, 1));
	st_sacrum_gndsacrum[0].setFunction(new LinearFunction());
	st_sacrum_gndsacrum[0].setAxis(Vec3(0, 0, 1));
	st_sacrum_gndsacrum[1].setCoordinateNames(OpenSim::Array<std::string>("sacrum_list", 1, 1));
	st_sacrum_gndsacrum[1].setFunction(new LinearFunction());
	st_sacrum_gndsacrum[1].setAxis(Vec3(1, 0, 0));
	st_sacrum_gndsacrum[2].setCoordinateNames(OpenSim::Array<std::string>("sacrum_rotation", 1, 1));
	st_sacrum_gndsacrum[2].setFunction(new LinearFunction());
	st_sacrum_gndsacrum[2].setAxis(Vec3(0, 1, 0));
	st_sacrum_gndsacrum[3].setCoordinateNames(OpenSim::Array<std::string>("sacrum_tx", 1, 1));
	st_sacrum_gndsacrum[3].setFunction(new LinearFunction());
	st_sacrum_gndsacrum[3].setAxis(Vec3(1, 0, 0));
	st_sacrum_gndsacrum[4].setCoordinateNames(OpenSim::Array<std::string>("sacrum_ty", 1, 1));
	st_sacrum_gndsacrum[4].setFunction(new LinearFunction());
	st_sacrum_gndsacrum[4].setAxis(Vec3(0, 1, 0));
	st_sacrum_gndsacrum[5].setCoordinateNames(OpenSim::Array<std::string>("sacrum_tz", 1, 1));
	st_sacrum_gndsacrum[5].setFunction(new LinearFunction());
	st_sacrum_gndsacrum[5].setAxis(Vec3(0, 0, 1));
	gndsacrum = new CustomJoint("gndsacrum", model->getGround(), Vec3(0, 0.89, 0), Vec3(0, 0, 0), *sacrum, Vec3(-0.070633, -0.026487, 0), Vec3(0, 0, 0), st_sacrum_gndsacrum);
	model->addJoint(gndsacrum);
	//
	//
	Coordinate& sacrum_tilt = gndsacrum->upd_coordinates(0);
	sacrum_tilt.setDefaultLocked(false);
	sacrum_tilt.setDefaultClamped(false);
	//
	Coordinate& sacrum_list = gndsacrum->upd_coordinates(1);
	sacrum_list.setDefaultLocked(false);
	sacrum_list.setDefaultClamped(false);
	//
	Coordinate& sacrum_rotation = gndsacrum->upd_coordinates(2);
	sacrum_rotation.setDefaultLocked(false);
	sacrum_rotation.setDefaultClamped(false);
	//
	Coordinate& sacrum_tx = gndsacrum->upd_coordinates(3);
	sacrum_tx.setDefaultLocked(false);
	sacrum_tx.setDefaultClamped(false);
	//
	Coordinate& sacrum_ty = gndsacrum->upd_coordinates(4);
	sacrum_ty.setDefaultLocked(false);
	sacrum_ty.setDefaultClamped(false);
	//
	Coordinate& sacrum_tz = gndsacrum->upd_coordinates(5);
	sacrum_tz.setDefaultLocked(false);
	sacrum_tz.setDefaultClamped(false);
	model->updJointSet();
	//
	sacrum_pelvis_weld = new WeldJoint("sacrum_pelvis_weld", *sacrum, Vec3(0, 0.043263, 0), Vec3(0, 0, 0), *pelvis, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(sacrum_pelvis_weld);
	//
	SpatialTransform st_Abdomen_Abdjnt;
	st_Abdomen_Abdjnt[0].setCoordinateNames(OpenSim::Array<std::string>("Abs_FE", 1, 1));
	st_Abdomen_Abdjnt[0].setFunction(new LinearFunction());
	st_Abdomen_Abdjnt[0].setAxis(Vec3(0, 0, 1));
	st_Abdomen_Abdjnt[1].setCoordinateNames(OpenSim::Array<std::string>("Abs_LB", 1, 1));
	st_Abdomen_Abdjnt[1].setFunction(new LinearFunction());
	st_Abdomen_Abdjnt[1].setAxis(Vec3(1, 0, 0));
	st_Abdomen_Abdjnt[2].setCoordinateNames(OpenSim::Array<std::string>("Abs_AR", 1, 1));
	st_Abdomen_Abdjnt[2].setFunction(new LinearFunction());
	st_Abdomen_Abdjnt[2].setAxis(Vec3(0, 1, 0));
	st_Abdomen_Abdjnt[3].setCoordinateNames(OpenSim::Array<std::string>("Abs_t1", 1, 1));
	st_Abdomen_Abdjnt[3].setFunction(new LinearFunction());
	st_Abdomen_Abdjnt[3].setAxis(Vec3(1, 0, 0));
	st_Abdomen_Abdjnt[4].setCoordinateNames(OpenSim::Array<std::string>("Abs_t2", 1, 1));
	st_Abdomen_Abdjnt[4].setFunction(new LinearFunction());
	st_Abdomen_Abdjnt[4].setAxis(Vec3(0, 1, 0));
	Abdjnt = new CustomJoint("Abdjnt", *sacrum, Vec3(-0.068867, 0.062687, 0), Vec3(0, 0, 0), *Abdomen, Vec3(-0.089785, -0.089785, 0), Vec3(0, 0, 0), st_Abdomen_Abdjnt);
	model->addJoint(Abdjnt);
	//
	//
	Coordinate& Abs_FE = Abdjnt->upd_coordinates(0);
	Abs_FE.setDefaultLocked(false);
	Abs_FE.setDefaultClamped(false);
	//
	Coordinate& Abs_LB = Abdjnt->upd_coordinates(1);
	Abs_LB.setDefaultLocked(false);
	Abs_LB.setDefaultClamped(false);
	//
	Coordinate& Abs_AR = Abdjnt->upd_coordinates(2);
	Abs_AR.setDefaultLocked(false);
	Abs_AR.setDefaultClamped(false);
	//
	Coordinate& Abs_t1 = Abdjnt->upd_coordinates(3);
	Abs_t1.setDefaultLocked(true);
	Abs_t1.setDefaultClamped(false);
	//
	Coordinate& Abs_t2 = Abdjnt->upd_coordinates(4);
	Abs_t2.setDefaultLocked(true);
	Abs_t2.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_lumbar5_L5_S1_IVDjnt;
	st_lumbar5_L5_S1_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("L5_S1_FE", 1, 1));
	st_lumbar5_L5_S1_IVDjnt[0].setFunction(new LinearFunction());
	st_lumbar5_L5_S1_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_lumbar5_L5_S1_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("L5_S1_LB", 1, 1));
	st_lumbar5_L5_S1_IVDjnt[1].setFunction(new LinearFunction());
	st_lumbar5_L5_S1_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_lumbar5_L5_S1_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("L5_S1_AR", 1, 1));
	st_lumbar5_L5_S1_IVDjnt[2].setFunction(new LinearFunction());
	st_lumbar5_L5_S1_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_lumbar5_L5_S1_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("L5_S1_Tx", 1, 1));
	st_lumbar5_L5_S1_IVDjnt[3].setFunction(new LinearFunction());
	st_lumbar5_L5_S1_IVDjnt[3].setAxis(Vec3(0.9551, -0.29628, 0));
	st_lumbar5_L5_S1_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("L5_S1_Ty", 1, 1));
	st_lumbar5_L5_S1_IVDjnt[4].setFunction(new LinearFunction());
	st_lumbar5_L5_S1_IVDjnt[4].setAxis(Vec3(0.29628, 0.9551, 0));
	st_lumbar5_L5_S1_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("L5_S1_Tz", 1, 1));
	st_lumbar5_L5_S1_IVDjnt[5].setFunction(new LinearFunction());
	st_lumbar5_L5_S1_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	L5_S1_IVDjnt = new CustomJoint("L5_S1_IVDjnt", *sacrum, Vec3(-0.086525, 0.062687, 0), Vec3(0, 0, -0.36652), *lumbar5, Vec3(0, 0, 0), Vec3(0, 0, 0), st_lumbar5_L5_S1_IVDjnt);
	model->addJoint(L5_S1_IVDjnt);
	//
	//
	Coordinate& L5_S1_FE = L5_S1_IVDjnt->upd_coordinates(0);
	L5_S1_FE.setDefaultLocked(false);
	L5_S1_FE.setDefaultClamped(false);
	//
	Coordinate& L5_S1_LB = L5_S1_IVDjnt->upd_coordinates(1);
	L5_S1_LB.setDefaultLocked(false);
	L5_S1_LB.setDefaultClamped(false);
	//
	Coordinate& L5_S1_AR = L5_S1_IVDjnt->upd_coordinates(2);
	L5_S1_AR.setDefaultLocked(false);
	L5_S1_AR.setDefaultClamped(false);
	//
	Coordinate& L5_S1_Tx = L5_S1_IVDjnt->upd_coordinates(3);
	L5_S1_Tx.setDefaultLocked(false);
	L5_S1_Tx.setDefaultClamped(true);
	//
	Coordinate& L5_S1_Ty = L5_S1_IVDjnt->upd_coordinates(4);
	L5_S1_Ty.setDefaultLocked(false);
	L5_S1_Ty.setDefaultClamped(true);
	//
	Coordinate& L5_S1_Tz = L5_S1_IVDjnt->upd_coordinates(5);
	L5_S1_Tz.setDefaultLocked(false);
	L5_S1_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_lumbar4_L4_L5_IVDjnt;
	st_lumbar4_L4_L5_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("L4_L5_FE", 1, 1));
	st_lumbar4_L4_L5_IVDjnt[0].setFunction(new LinearFunction());
	st_lumbar4_L4_L5_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_lumbar4_L4_L5_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("L4_L5_LB", 1, 1));
	st_lumbar4_L4_L5_IVDjnt[1].setFunction(new LinearFunction());
	st_lumbar4_L4_L5_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_lumbar4_L4_L5_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("L4_L5_AR", 1, 1));
	st_lumbar4_L4_L5_IVDjnt[2].setFunction(new LinearFunction());
	st_lumbar4_L4_L5_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_lumbar4_L4_L5_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("L4_L5_Tx", 1, 1));
	st_lumbar4_L4_L5_IVDjnt[3].setFunction(new LinearFunction());
	st_lumbar4_L4_L5_IVDjnt[3].setAxis(Vec3(0.99777, -0.066805, 0));
	st_lumbar4_L4_L5_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("L4_L5_Ty", 1, 1));
	st_lumbar4_L4_L5_IVDjnt[4].setFunction(new LinearFunction());
	st_lumbar4_L4_L5_IVDjnt[4].setAxis(Vec3(0.066805, 0.99777, 0));
	st_lumbar4_L4_L5_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("L4_L5_Tz", 1, 1));
	st_lumbar4_L4_L5_IVDjnt[5].setFunction(new LinearFunction());
	st_lumbar4_L4_L5_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	L4_L5_IVDjnt = new CustomJoint("L4_L5_IVDjnt", *lumbar5, Vec3(0, 0.034947, 0), Vec3(0, 0, 0.31416), *lumbar4, Vec3(0, 0, 0), Vec3(0, 0, 0), st_lumbar4_L4_L5_IVDjnt);
	model->addJoint(L4_L5_IVDjnt);
	//
	//
	Coordinate& L4_L5_FE = L4_L5_IVDjnt->upd_coordinates(0);
	L4_L5_FE.setDefaultLocked(false);
	L4_L5_FE.setDefaultClamped(false);
	//
	Coordinate& L4_L5_LB = L4_L5_IVDjnt->upd_coordinates(1);
	L4_L5_LB.setDefaultLocked(false);
	L4_L5_LB.setDefaultClamped(false);
	//
	Coordinate& L4_L5_AR = L4_L5_IVDjnt->upd_coordinates(2);
	L4_L5_AR.setDefaultLocked(false);
	L4_L5_AR.setDefaultClamped(false);
	//
	Coordinate& L4_L5_Tx = L4_L5_IVDjnt->upd_coordinates(3);
	L4_L5_Tx.setDefaultLocked(false);
	L4_L5_Tx.setDefaultClamped(true);
	//
	Coordinate& L4_L5_Ty = L4_L5_IVDjnt->upd_coordinates(4);
	L4_L5_Ty.setDefaultLocked(false);
	L4_L5_Ty.setDefaultClamped(true);
	//
	Coordinate& L4_L5_Tz = L4_L5_IVDjnt->upd_coordinates(5);
	L4_L5_Tz.setDefaultLocked(false);
	L4_L5_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_lumbar3_L3_L4_IVDjnt;
	st_lumbar3_L3_L4_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("L3_L4_FE", 1, 1));
	st_lumbar3_L3_L4_IVDjnt[0].setFunction(new LinearFunction());
	st_lumbar3_L3_L4_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_lumbar3_L3_L4_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("L3_L4_LB", 1, 1));
	st_lumbar3_L3_L4_IVDjnt[1].setFunction(new LinearFunction());
	st_lumbar3_L3_L4_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_lumbar3_L3_L4_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("L3_L4_AR", 1, 1));
	st_lumbar3_L3_L4_IVDjnt[2].setFunction(new LinearFunction());
	st_lumbar3_L3_L4_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_lumbar3_L3_L4_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("L3_L4_Tx", 1, 1));
	st_lumbar3_L3_L4_IVDjnt[3].setFunction(new LinearFunction());
	st_lumbar3_L3_L4_IVDjnt[3].setAxis(Vec3(0.98651, 0.16373, 0));
	st_lumbar3_L3_L4_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("L3_L4_Ty", 1, 1));
	st_lumbar3_L3_L4_IVDjnt[4].setFunction(new LinearFunction());
	st_lumbar3_L3_L4_IVDjnt[4].setAxis(Vec3(-0.16373, 0.98651, 0));
	st_lumbar3_L3_L4_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("L3_L4_Tz", 1, 1));
	st_lumbar3_L3_L4_IVDjnt[5].setFunction(new LinearFunction());
	st_lumbar3_L3_L4_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	L3_L4_IVDjnt = new CustomJoint("L3_L4_IVDjnt", *lumbar4, Vec3(0, 0.032493, 0), Vec3(0, 0, 0.20944), *lumbar3, Vec3(0, 0, 0), Vec3(0, 0, 0), st_lumbar3_L3_L4_IVDjnt);
	model->addJoint(L3_L4_IVDjnt);
	//
	//
	Coordinate& L3_L4_FE = L3_L4_IVDjnt->upd_coordinates(0);
	L3_L4_FE.setDefaultLocked(false);
	L3_L4_FE.setDefaultClamped(false);
	//
	Coordinate& L3_L4_LB = L3_L4_IVDjnt->upd_coordinates(1);
	L3_L4_LB.setDefaultLocked(false);
	L3_L4_LB.setDefaultClamped(false);
	//
	Coordinate& L3_L4_AR = L3_L4_IVDjnt->upd_coordinates(2);
	L3_L4_AR.setDefaultLocked(false);
	L3_L4_AR.setDefaultClamped(false);
	//
	Coordinate& L3_L4_Tx = L3_L4_IVDjnt->upd_coordinates(3);
	L3_L4_Tx.setDefaultLocked(false);
	L3_L4_Tx.setDefaultClamped(true);
	//
	Coordinate& L3_L4_Ty = L3_L4_IVDjnt->upd_coordinates(4);
	L3_L4_Ty.setDefaultLocked(false);
	L3_L4_Ty.setDefaultClamped(true);
	//
	Coordinate& L3_L4_Tz = L3_L4_IVDjnt->upd_coordinates(5);
	L3_L4_Tz.setDefaultLocked(false);
	L3_L4_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_lumbar2_L2_L3_IVDjnt;
	st_lumbar2_L2_L3_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("L2_L3_FE", 1, 1));
	st_lumbar2_L2_L3_IVDjnt[0].setFunction(new LinearFunction());
	st_lumbar2_L2_L3_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_lumbar2_L2_L3_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("L2_L3_LB", 1, 1));
	st_lumbar2_L2_L3_IVDjnt[1].setFunction(new LinearFunction());
	st_lumbar2_L2_L3_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_lumbar2_L2_L3_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("L2_L3_AR", 1, 1));
	st_lumbar2_L2_L3_IVDjnt[2].setFunction(new LinearFunction());
	st_lumbar2_L2_L3_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_lumbar2_L2_L3_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("L2_L3_Tx", 1, 1));
	st_lumbar2_L2_L3_IVDjnt[3].setFunction(new LinearFunction());
	st_lumbar2_L2_L3_IVDjnt[3].setAxis(Vec3(0.94458, 0.32827, 0));
	st_lumbar2_L2_L3_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("L2_L3_Ty", 1, 1));
	st_lumbar2_L2_L3_IVDjnt[4].setFunction(new LinearFunction());
	st_lumbar2_L2_L3_IVDjnt[4].setAxis(Vec3(-0.32827, 0.94458, 0));
	st_lumbar2_L2_L3_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("L2_L3_Tz", 1, 1));
	st_lumbar2_L2_L3_IVDjnt[5].setFunction(new LinearFunction());
	st_lumbar2_L2_L3_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	L2_L3_IVDjnt = new CustomJoint("L2_L3_IVDjnt", *lumbar3, Vec3(0, 0.032459, 0), Vec3(0, 0, 0.12217), *lumbar2, Vec3(0, 0, 0), Vec3(0, 0, 0), st_lumbar2_L2_L3_IVDjnt);
	model->addJoint(L2_L3_IVDjnt);
	//
	//
	Coordinate& L2_L3_FE = L2_L3_IVDjnt->upd_coordinates(0);
	L2_L3_FE.setDefaultLocked(false);
	L2_L3_FE.setDefaultClamped(false);
	//
	Coordinate& L2_L3_LB = L2_L3_IVDjnt->upd_coordinates(1);
	L2_L3_LB.setDefaultLocked(false);
	L2_L3_LB.setDefaultClamped(false);
	//
	Coordinate& L2_L3_AR = L2_L3_IVDjnt->upd_coordinates(2);
	L2_L3_AR.setDefaultLocked(false);
	L2_L3_AR.setDefaultClamped(false);
	//
	Coordinate& L2_L3_Tx = L2_L3_IVDjnt->upd_coordinates(3);
	L2_L3_Tx.setDefaultLocked(false);
	L2_L3_Tx.setDefaultClamped(true);
	//
	Coordinate& L2_L3_Ty = L2_L3_IVDjnt->upd_coordinates(4);
	L2_L3_Ty.setDefaultLocked(false);
	L2_L3_Ty.setDefaultClamped(true);
	//
	Coordinate& L2_L3_Tz = L2_L3_IVDjnt->upd_coordinates(5);
	L2_L3_Tz.setDefaultLocked(false);
	L2_L3_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_lumbar1_L1_L2_IVDjnt;
	st_lumbar1_L1_L2_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("L1_L2_FE", 1, 1));
	st_lumbar1_L1_L2_IVDjnt[0].setFunction(new LinearFunction());
	st_lumbar1_L1_L2_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_lumbar1_L1_L2_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("L1_L2_LB", 1, 1));
	st_lumbar1_L1_L2_IVDjnt[1].setFunction(new LinearFunction());
	st_lumbar1_L1_L2_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_lumbar1_L1_L2_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("L1_L2_AR", 1, 1));
	st_lumbar1_L1_L2_IVDjnt[2].setFunction(new LinearFunction());
	st_lumbar1_L1_L2_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_lumbar1_L1_L2_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("L1_L2_Tx", 1, 1));
	st_lumbar1_L1_L2_IVDjnt[3].setFunction(new LinearFunction());
	st_lumbar1_L1_L2_IVDjnt[3].setAxis(Vec3(0.91532, 0.40272, 0));
	st_lumbar1_L1_L2_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("L1_L2_Ty", 1, 1));
	st_lumbar1_L1_L2_IVDjnt[4].setFunction(new LinearFunction());
	st_lumbar1_L1_L2_IVDjnt[4].setAxis(Vec3(-0.40272, 0.91532, 0));
	st_lumbar1_L1_L2_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("L1_L2_Tz", 1, 1));
	st_lumbar1_L1_L2_IVDjnt[5].setFunction(new LinearFunction());
	st_lumbar1_L1_L2_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	L1_L2_IVDjnt = new CustomJoint("L1_L2_IVDjnt", *lumbar2, Vec3(0, 0.030067, 0), Vec3(0, 0, 0.10472), *lumbar1, Vec3(0, 0, 0), Vec3(0, 0, 0), st_lumbar1_L1_L2_IVDjnt);
	model->addJoint(L1_L2_IVDjnt);
	//
	//
	Coordinate& L1_L2_FE = L1_L2_IVDjnt->upd_coordinates(0);
	L1_L2_FE.setDefaultLocked(false);
	L1_L2_FE.setDefaultClamped(false);
	//
	Coordinate& L1_L2_LB = L1_L2_IVDjnt->upd_coordinates(1);
	L1_L2_LB.setDefaultLocked(false);
	L1_L2_LB.setDefaultClamped(false);
	//
	Coordinate& L1_L2_AR = L1_L2_IVDjnt->upd_coordinates(2);
	L1_L2_AR.setDefaultLocked(false);
	L1_L2_AR.setDefaultClamped(false);
	//
	Coordinate& L1_L2_Tx = L1_L2_IVDjnt->upd_coordinates(3);
	L1_L2_Tx.setDefaultLocked(false);
	L1_L2_Tx.setDefaultClamped(true);
	//
	Coordinate& L1_L2_Ty = L1_L2_IVDjnt->upd_coordinates(4);
	L1_L2_Ty.setDefaultLocked(false);
	L1_L2_Ty.setDefaultClamped(true);
	//
	Coordinate& L1_L2_Tz = L1_L2_IVDjnt->upd_coordinates(5);
	L1_L2_Tz.setDefaultLocked(false);
	L1_L2_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic12_T12_L1_IVDjnt;
	st_thoracic12_T12_L1_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T12_L1_FE", 1, 1));
	st_thoracic12_T12_L1_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic12_T12_L1_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic12_T12_L1_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T12_L1_LB", 1, 1));
	st_thoracic12_T12_L1_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic12_T12_L1_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic12_T12_L1_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T12_L1_AR", 1, 1));
	st_thoracic12_T12_L1_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic12_T12_L1_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic12_T12_L1_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T12_L1_Tx", 1, 1));
	st_thoracic12_T12_L1_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic12_T12_L1_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic12_T12_L1_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T12_L1_Ty", 1, 1));
	st_thoracic12_T12_L1_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic12_T12_L1_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic12_T12_L1_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T12_L1_Tz", 1, 1));
	st_thoracic12_T12_L1_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic12_T12_L1_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T12_L1_IVDjnt = new CustomJoint("T12_L1_IVDjnt", *lumbar1, Vec3(0, 0.031341, 0), Vec3(0, 0, -0.034907), *thoracic12, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic12_T12_L1_IVDjnt);
	model->addJoint(T12_L1_IVDjnt);
	//
	//
	Coordinate& T12_L1_FE = T12_L1_IVDjnt->upd_coordinates(0);
	T12_L1_FE.setDefaultLocked(false);
	T12_L1_FE.setDefaultClamped(false);
	//
	Coordinate& T12_L1_LB = T12_L1_IVDjnt->upd_coordinates(1);
	T12_L1_LB.setDefaultLocked(false);
	T12_L1_LB.setDefaultClamped(false);
	//
	Coordinate& T12_L1_AR = T12_L1_IVDjnt->upd_coordinates(2);
	T12_L1_AR.setDefaultLocked(false);
	T12_L1_AR.setDefaultClamped(false);
	//
	Coordinate& T12_L1_Tx = T12_L1_IVDjnt->upd_coordinates(3);
	T12_L1_Tx.setDefaultLocked(false);
	T12_L1_Tx.setDefaultClamped(true);
	//
	Coordinate& T12_L1_Ty = T12_L1_IVDjnt->upd_coordinates(4);
	T12_L1_Ty.setDefaultLocked(false);
	T12_L1_Ty.setDefaultClamped(true);
	//
	Coordinate& T12_L1_Tz = T12_L1_IVDjnt->upd_coordinates(5);
	T12_L1_Tz.setDefaultLocked(false);
	T12_L1_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic11_T11_T12_IVDjnt;
	st_thoracic11_T11_T12_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T11_T12_FE", 1, 1));
	st_thoracic11_T11_T12_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic11_T11_T12_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic11_T11_T12_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T11_T12_LB", 1, 1));
	st_thoracic11_T11_T12_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic11_T11_T12_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic11_T11_T12_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T11_T12_AR", 1, 1));
	st_thoracic11_T11_T12_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic11_T11_T12_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic11_T11_T12_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T11_T12_Tx", 1, 1));
	st_thoracic11_T11_T12_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic11_T11_T12_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic11_T11_T12_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T11_T12_Ty", 1, 1));
	st_thoracic11_T11_T12_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic11_T11_T12_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic11_T11_T12_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T11_T12_Tz", 1, 1));
	st_thoracic11_T11_T12_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic11_T11_T12_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T11_T12_IVDjnt = new CustomJoint("T11_T12_IVDjnt", *thoracic12, Vec3(0, 0.033227, 0), Vec3(0, 0, -0.069813), *thoracic11, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic11_T11_T12_IVDjnt);
	model->addJoint(T11_T12_IVDjnt);
	//
	//
	Coordinate& T11_T12_FE = T11_T12_IVDjnt->upd_coordinates(0);
	T11_T12_FE.setDefaultLocked(false);
	T11_T12_FE.setDefaultClamped(false);
	//
	Coordinate& T11_T12_LB = T11_T12_IVDjnt->upd_coordinates(1);
	T11_T12_LB.setDefaultLocked(false);
	T11_T12_LB.setDefaultClamped(false);
	//
	Coordinate& T11_T12_AR = T11_T12_IVDjnt->upd_coordinates(2);
	T11_T12_AR.setDefaultLocked(false);
	T11_T12_AR.setDefaultClamped(false);
	//
	Coordinate& T11_T12_Tx = T11_T12_IVDjnt->upd_coordinates(3);
	T11_T12_Tx.setDefaultLocked(false);
	T11_T12_Tx.setDefaultClamped(true);
	//
	Coordinate& T11_T12_Ty = T11_T12_IVDjnt->upd_coordinates(4);
	T11_T12_Ty.setDefaultLocked(false);
	T11_T12_Ty.setDefaultClamped(true);
	//
	Coordinate& T11_T12_Tz = T11_T12_IVDjnt->upd_coordinates(5);
	T11_T12_Tz.setDefaultLocked(false);
	T11_T12_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic10_T10_T11_IVDjnt;
	st_thoracic10_T10_T11_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T10_T11_FE", 1, 1));
	st_thoracic10_T10_T11_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic10_T10_T11_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic10_T10_T11_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T10_T11_LB", 1, 1));
	st_thoracic10_T10_T11_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic10_T10_T11_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic10_T10_T11_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T10_T11_AR", 1, 1));
	st_thoracic10_T10_T11_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic10_T10_T11_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic10_T10_T11_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T10_T11_Tx", 1, 1));
	st_thoracic10_T10_T11_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic10_T10_T11_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic10_T10_T11_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T10_T11_Ty", 1, 1));
	st_thoracic10_T10_T11_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic10_T10_T11_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic10_T10_T11_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T10_T11_Tz", 1, 1));
	st_thoracic10_T10_T11_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic10_T10_T11_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T10_T11_IVDjnt = new CustomJoint("T10_T11_IVDjnt", *thoracic11, Vec3(0, 0.029012, 0), Vec3(0, 0, -0.087266), *thoracic10, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic10_T10_T11_IVDjnt);
	model->addJoint(T10_T11_IVDjnt);
	//
	//
	Coordinate& T10_T11_FE = T10_T11_IVDjnt->upd_coordinates(0);
	T10_T11_FE.setDefaultLocked(false);
	T10_T11_FE.setDefaultClamped(false);
	//
	Coordinate& T10_T11_LB = T10_T11_IVDjnt->upd_coordinates(1);
	T10_T11_LB.setDefaultLocked(false);
	T10_T11_LB.setDefaultClamped(false);
	//
	Coordinate& T10_T11_AR = T10_T11_IVDjnt->upd_coordinates(2);
	T10_T11_AR.setDefaultLocked(false);
	T10_T11_AR.setDefaultClamped(false);
	//
	Coordinate& T10_T11_Tx = T10_T11_IVDjnt->upd_coordinates(3);
	T10_T11_Tx.setDefaultLocked(false);
	T10_T11_Tx.setDefaultClamped(true);
	//
	Coordinate& T10_T11_Ty = T10_T11_IVDjnt->upd_coordinates(4);
	T10_T11_Ty.setDefaultLocked(false);
	T10_T11_Ty.setDefaultClamped(true);
	//
	Coordinate& T10_T11_Tz = T10_T11_IVDjnt->upd_coordinates(5);
	T10_T11_Tz.setDefaultLocked(false);
	T10_T11_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic9_T9_T10_IVDjnt;
	st_thoracic9_T9_T10_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T9_T10_FE", 1, 1));
	st_thoracic9_T9_T10_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic9_T9_T10_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic9_T9_T10_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T9_T10_LB", 1, 1));
	st_thoracic9_T9_T10_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic9_T9_T10_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic9_T9_T10_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T9_T10_AR", 1, 1));
	st_thoracic9_T9_T10_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic9_T9_T10_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic9_T9_T10_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T9_T10_Tx", 1, 1));
	st_thoracic9_T9_T10_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic9_T9_T10_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic9_T9_T10_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T9_T10_Ty", 1, 1));
	st_thoracic9_T9_T10_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic9_T9_T10_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic9_T9_T10_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T9_T10_Tz", 1, 1));
	st_thoracic9_T9_T10_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic9_T9_T10_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T9_T10_IVDjnt = new CustomJoint("T9_T10_IVDjnt", *thoracic10, Vec3(0, 0.028477, 0), Vec3(0, 0, -0.069813), *thoracic9, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic9_T9_T10_IVDjnt);
	model->addJoint(T9_T10_IVDjnt);
	//
	//
	Coordinate& T9_T10_FE = T9_T10_IVDjnt->upd_coordinates(0);
	T9_T10_FE.setDefaultLocked(false);
	T9_T10_FE.setDefaultClamped(false);
	//
	Coordinate& T9_T10_LB = T9_T10_IVDjnt->upd_coordinates(1);
	T9_T10_LB.setDefaultLocked(false);
	T9_T10_LB.setDefaultClamped(false);
	//
	Coordinate& T9_T10_AR = T9_T10_IVDjnt->upd_coordinates(2);
	T9_T10_AR.setDefaultLocked(false);
	T9_T10_AR.setDefaultClamped(false);
	//
	Coordinate& T9_T10_Tx = T9_T10_IVDjnt->upd_coordinates(3);
	T9_T10_Tx.setDefaultLocked(false);
	T9_T10_Tx.setDefaultClamped(true);
	//
	Coordinate& T9_T10_Ty = T9_T10_IVDjnt->upd_coordinates(4);
	T9_T10_Ty.setDefaultLocked(false);
	T9_T10_Ty.setDefaultClamped(true);
	//
	Coordinate& T9_T10_Tz = T9_T10_IVDjnt->upd_coordinates(5);
	T9_T10_Tz.setDefaultLocked(false);
	T9_T10_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic8_T8_T9_IVDjnt;
	st_thoracic8_T8_T9_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T8_T9_FE", 1, 1));
	st_thoracic8_T8_T9_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic8_T8_T9_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic8_T8_T9_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T8_T9_LB", 1, 1));
	st_thoracic8_T8_T9_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic8_T8_T9_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic8_T8_T9_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T8_T9_AR", 1, 1));
	st_thoracic8_T8_T9_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic8_T8_T9_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic8_T8_T9_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T8_T9_Tx", 1, 1));
	st_thoracic8_T8_T9_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic8_T8_T9_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic8_T8_T9_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T8_T9_Ty", 1, 1));
	st_thoracic8_T8_T9_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic8_T8_T9_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic8_T8_T9_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T8_T9_Tz", 1, 1));
	st_thoracic8_T8_T9_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic8_T8_T9_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T8_T9_IVDjnt = new CustomJoint("T8_T9_IVDjnt", *thoracic9, Vec3(0, 0.024682, 0), Vec3(0, 0, -0.087266), *thoracic8, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic8_T8_T9_IVDjnt);
	model->addJoint(T8_T9_IVDjnt);
	//
	//
	Coordinate& T8_T9_FE = T8_T9_IVDjnt->upd_coordinates(0);
	T8_T9_FE.setDefaultLocked(false);
	T8_T9_FE.setDefaultClamped(false);
	//
	Coordinate& T8_T9_LB = T8_T9_IVDjnt->upd_coordinates(1);
	T8_T9_LB.setDefaultLocked(false);
	T8_T9_LB.setDefaultClamped(false);
	//
	Coordinate& T8_T9_AR = T8_T9_IVDjnt->upd_coordinates(2);
	T8_T9_AR.setDefaultLocked(false);
	T8_T9_AR.setDefaultClamped(false);
	//
	Coordinate& T8_T9_Tx = T8_T9_IVDjnt->upd_coordinates(3);
	T8_T9_Tx.setDefaultLocked(false);
	T8_T9_Tx.setDefaultClamped(true);
	//
	Coordinate& T8_T9_Ty = T8_T9_IVDjnt->upd_coordinates(4);
	T8_T9_Ty.setDefaultLocked(false);
	T8_T9_Ty.setDefaultClamped(true);
	//
	Coordinate& T8_T9_Tz = T8_T9_IVDjnt->upd_coordinates(5);
	T8_T9_Tz.setDefaultLocked(false);
	T8_T9_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic7_T7_T8_IVDjnt;
	st_thoracic7_T7_T8_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T7_T8_FE", 1, 1));
	st_thoracic7_T7_T8_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic7_T7_T8_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic7_T7_T8_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T7_T8_LB", 1, 1));
	st_thoracic7_T7_T8_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic7_T7_T8_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic7_T7_T8_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T7_T8_AR", 1, 1));
	st_thoracic7_T7_T8_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic7_T7_T8_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic7_T7_T8_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T7_T8_Tx", 1, 1));
	st_thoracic7_T7_T8_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic7_T7_T8_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic7_T7_T8_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T7_T8_Ty", 1, 1));
	st_thoracic7_T7_T8_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic7_T7_T8_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic7_T7_T8_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T7_T8_Tz", 1, 1));
	st_thoracic7_T7_T8_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic7_T7_T8_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T7_T8_IVDjnt = new CustomJoint("T7_T8_IVDjnt", *thoracic8, Vec3(0, 0.024171, 0), Vec3(0, 0, -0.10472), *thoracic7, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic7_T7_T8_IVDjnt);
	model->addJoint(T7_T8_IVDjnt);
	//
	//
	Coordinate& T7_T8_FE = T7_T8_IVDjnt->upd_coordinates(0);
	T7_T8_FE.setDefaultLocked(false);
	T7_T8_FE.setDefaultClamped(false);
	//
	Coordinate& T7_T8_LB = T7_T8_IVDjnt->upd_coordinates(1);
	T7_T8_LB.setDefaultLocked(false);
	T7_T8_LB.setDefaultClamped(false);
	//
	Coordinate& T7_T8_AR = T7_T8_IVDjnt->upd_coordinates(2);
	T7_T8_AR.setDefaultLocked(false);
	T7_T8_AR.setDefaultClamped(false);
	//
	Coordinate& T7_T8_Tx = T7_T8_IVDjnt->upd_coordinates(3);
	T7_T8_Tx.setDefaultLocked(false);
	T7_T8_Tx.setDefaultClamped(true);
	//
	Coordinate& T7_T8_Ty = T7_T8_IVDjnt->upd_coordinates(4);
	T7_T8_Ty.setDefaultLocked(false);
	T7_T8_Ty.setDefaultClamped(true);
	//
	Coordinate& T7_T8_Tz = T7_T8_IVDjnt->upd_coordinates(5);
	T7_T8_Tz.setDefaultLocked(false);
	T7_T8_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic6_T6_T7_IVDjnt;
	st_thoracic6_T6_T7_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T6_T7_FE", 1, 1));
	st_thoracic6_T6_T7_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic6_T6_T7_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic6_T6_T7_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T6_T7_LB", 1, 1));
	st_thoracic6_T6_T7_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic6_T6_T7_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic6_T6_T7_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T6_T7_AR", 1, 1));
	st_thoracic6_T6_T7_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic6_T6_T7_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic6_T6_T7_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T6_T7_Tx", 1, 1));
	st_thoracic6_T6_T7_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic6_T6_T7_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic6_T6_T7_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T6_T7_Ty", 1, 1));
	st_thoracic6_T6_T7_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic6_T6_T7_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic6_T6_T7_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T6_T7_Tz", 1, 1));
	st_thoracic6_T6_T7_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic6_T6_T7_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T6_T7_IVDjnt = new CustomJoint("T6_T7_IVDjnt", *thoracic7, Vec3(0, 0.025467, 0), Vec3(0, 0, -0.10472), *thoracic6, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic6_T6_T7_IVDjnt);
	model->addJoint(T6_T7_IVDjnt);
	//
	//
	Coordinate& T6_T7_FE = T6_T7_IVDjnt->upd_coordinates(0);
	T6_T7_FE.setDefaultLocked(false);
	T6_T7_FE.setDefaultClamped(false);
	//
	Coordinate& T6_T7_LB = T6_T7_IVDjnt->upd_coordinates(1);
	T6_T7_LB.setDefaultLocked(false);
	T6_T7_LB.setDefaultClamped(false);
	//
	Coordinate& T6_T7_AR = T6_T7_IVDjnt->upd_coordinates(2);
	T6_T7_AR.setDefaultLocked(false);
	T6_T7_AR.setDefaultClamped(false);
	//
	Coordinate& T6_T7_Tx = T6_T7_IVDjnt->upd_coordinates(3);
	T6_T7_Tx.setDefaultLocked(false);
	T6_T7_Tx.setDefaultClamped(true);
	//
	Coordinate& T6_T7_Ty = T6_T7_IVDjnt->upd_coordinates(4);
	T6_T7_Ty.setDefaultLocked(false);
	T6_T7_Ty.setDefaultClamped(true);
	//
	Coordinate& T6_T7_Tz = T6_T7_IVDjnt->upd_coordinates(5);
	T6_T7_Tz.setDefaultLocked(false);
	T6_T7_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic5_T5_T6_IVDjnt;
	st_thoracic5_T5_T6_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T5_T6_FE", 1, 1));
	st_thoracic5_T5_T6_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic5_T5_T6_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic5_T5_T6_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T5_T6_LB", 1, 1));
	st_thoracic5_T5_T6_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic5_T5_T6_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic5_T5_T6_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T5_T6_AR", 1, 1));
	st_thoracic5_T5_T6_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic5_T5_T6_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic5_T5_T6_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T5_T6_Tx", 1, 1));
	st_thoracic5_T5_T6_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic5_T5_T6_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic5_T5_T6_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T5_T6_Ty", 1, 1));
	st_thoracic5_T5_T6_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic5_T5_T6_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic5_T5_T6_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T5_T6_Tz", 1, 1));
	st_thoracic5_T5_T6_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic5_T5_T6_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T5_T6_IVDjnt = new CustomJoint("T5_T6_IVDjnt", *thoracic6, Vec3(0, 0.024198, 0), Vec3(0, 0, -0.10472), *thoracic5, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic5_T5_T6_IVDjnt);
	model->addJoint(T5_T6_IVDjnt);
	//
	//
	Coordinate& T5_T6_FE = T5_T6_IVDjnt->upd_coordinates(0);
	T5_T6_FE.setDefaultLocked(false);
	T5_T6_FE.setDefaultClamped(false);
	//
	Coordinate& T5_T6_LB = T5_T6_IVDjnt->upd_coordinates(1);
	T5_T6_LB.setDefaultLocked(false);
	T5_T6_LB.setDefaultClamped(false);
	//
	Coordinate& T5_T6_AR = T5_T6_IVDjnt->upd_coordinates(2);
	T5_T6_AR.setDefaultLocked(false);
	T5_T6_AR.setDefaultClamped(false);
	//
	Coordinate& T5_T6_Tx = T5_T6_IVDjnt->upd_coordinates(3);
	T5_T6_Tx.setDefaultLocked(false);
	T5_T6_Tx.setDefaultClamped(true);
	//
	Coordinate& T5_T6_Ty = T5_T6_IVDjnt->upd_coordinates(4);
	T5_T6_Ty.setDefaultLocked(false);
	T5_T6_Ty.setDefaultClamped(true);
	//
	Coordinate& T5_T6_Tz = T5_T6_IVDjnt->upd_coordinates(5);
	T5_T6_Tz.setDefaultLocked(false);
	T5_T6_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic4_T4_T5_IVDjnt;
	st_thoracic4_T4_T5_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T4_T5_FE", 1, 1));
	st_thoracic4_T4_T5_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic4_T4_T5_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic4_T4_T5_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T4_T5_LB", 1, 1));
	st_thoracic4_T4_T5_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic4_T4_T5_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic4_T4_T5_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T4_T5_AR", 1, 1));
	st_thoracic4_T4_T5_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic4_T4_T5_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic4_T4_T5_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T4_T5_Tx", 1, 1));
	st_thoracic4_T4_T5_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic4_T4_T5_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic4_T4_T5_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T4_T5_Ty", 1, 1));
	st_thoracic4_T4_T5_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic4_T4_T5_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic4_T4_T5_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T4_T5_Tz", 1, 1));
	st_thoracic4_T4_T5_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic4_T4_T5_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T4_T5_IVDjnt = new CustomJoint("T4_T5_IVDjnt", *thoracic5, Vec3(0, 0.024759, 0), Vec3(0, 0, -0.10472), *thoracic4, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic4_T4_T5_IVDjnt);
	model->addJoint(T4_T5_IVDjnt);
	//
	//
	Coordinate& T4_T5_FE = T4_T5_IVDjnt->upd_coordinates(0);
	T4_T5_FE.setDefaultLocked(false);
	T4_T5_FE.setDefaultClamped(false);
	//
	Coordinate& T4_T5_LB = T4_T5_IVDjnt->upd_coordinates(1);
	T4_T5_LB.setDefaultLocked(false);
	T4_T5_LB.setDefaultClamped(false);
	//
	Coordinate& T4_T5_AR = T4_T5_IVDjnt->upd_coordinates(2);
	T4_T5_AR.setDefaultLocked(false);
	T4_T5_AR.setDefaultClamped(false);
	//
	Coordinate& T4_T5_Tx = T4_T5_IVDjnt->upd_coordinates(3);
	T4_T5_Tx.setDefaultLocked(false);
	T4_T5_Tx.setDefaultClamped(true);
	//
	Coordinate& T4_T5_Ty = T4_T5_IVDjnt->upd_coordinates(4);
	T4_T5_Ty.setDefaultLocked(false);
	T4_T5_Ty.setDefaultClamped(true);
	//
	Coordinate& T4_T5_Tz = T4_T5_IVDjnt->upd_coordinates(5);
	T4_T5_Tz.setDefaultLocked(false);
	T4_T5_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic3_T3_T4_IVDjnt;
	st_thoracic3_T3_T4_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T3_T4_FE", 1, 1));
	st_thoracic3_T3_T4_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic3_T3_T4_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic3_T3_T4_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T3_T4_LB", 1, 1));
	st_thoracic3_T3_T4_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic3_T3_T4_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic3_T3_T4_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T3_T4_AR", 1, 1));
	st_thoracic3_T3_T4_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic3_T3_T4_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic3_T3_T4_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T3_T4_Tx", 1, 1));
	st_thoracic3_T3_T4_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic3_T3_T4_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic3_T3_T4_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T3_T4_Ty", 1, 1));
	st_thoracic3_T3_T4_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic3_T3_T4_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic3_T3_T4_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T3_T4_Tz", 1, 1));
	st_thoracic3_T3_T4_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic3_T3_T4_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T3_T4_IVDjnt = new CustomJoint("T3_T4_IVDjnt", *thoracic4, Vec3(0, 0.02307, 0), Vec3(0, 0, -0.069813), *thoracic3, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic3_T3_T4_IVDjnt);
	model->addJoint(T3_T4_IVDjnt);
	//
	//
	Coordinate& T3_T4_FE = T3_T4_IVDjnt->upd_coordinates(0);
	T3_T4_FE.setDefaultLocked(false);
	T3_T4_FE.setDefaultClamped(false);
	//
	Coordinate& T3_T4_LB = T3_T4_IVDjnt->upd_coordinates(1);
	T3_T4_LB.setDefaultLocked(false);
	T3_T4_LB.setDefaultClamped(false);
	//
	Coordinate& T3_T4_AR = T3_T4_IVDjnt->upd_coordinates(2);
	T3_T4_AR.setDefaultLocked(false);
	T3_T4_AR.setDefaultClamped(false);
	//
	Coordinate& T3_T4_Tx = T3_T4_IVDjnt->upd_coordinates(3);
	T3_T4_Tx.setDefaultLocked(false);
	T3_T4_Tx.setDefaultClamped(true);
	//
	Coordinate& T3_T4_Ty = T3_T4_IVDjnt->upd_coordinates(4);
	T3_T4_Ty.setDefaultLocked(false);
	T3_T4_Ty.setDefaultClamped(true);
	//
	Coordinate& T3_T4_Tz = T3_T4_IVDjnt->upd_coordinates(5);
	T3_T4_Tz.setDefaultLocked(false);
	T3_T4_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic2_T2_T3_IVDjnt;
	st_thoracic2_T2_T3_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T2_T3_FE", 1, 1));
	st_thoracic2_T2_T3_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic2_T2_T3_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic2_T2_T3_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T2_T3_LB", 1, 1));
	st_thoracic2_T2_T3_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic2_T2_T3_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic2_T2_T3_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T2_T3_AR", 1, 1));
	st_thoracic2_T2_T3_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic2_T2_T3_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic2_T2_T3_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T2_T3_Tx", 1, 1));
	st_thoracic2_T2_T3_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic2_T2_T3_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic2_T2_T3_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T2_T3_Ty", 1, 1));
	st_thoracic2_T2_T3_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic2_T2_T3_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic2_T2_T3_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T2_T3_Tz", 1, 1));
	st_thoracic2_T2_T3_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic2_T2_T3_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T2_T3_IVDjnt = new CustomJoint("T2_T3_IVDjnt", *thoracic3, Vec3(0, 0.020935, 0), Vec3(0, 0, -0.05236), *thoracic2, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic2_T2_T3_IVDjnt);
	model->addJoint(T2_T3_IVDjnt);
	//
	//
	Coordinate& T2_T3_FE = T2_T3_IVDjnt->upd_coordinates(0);
	T2_T3_FE.setDefaultLocked(false);
	T2_T3_FE.setDefaultClamped(false);
	//
	Coordinate& T2_T3_LB = T2_T3_IVDjnt->upd_coordinates(1);
	T2_T3_LB.setDefaultLocked(false);
	T2_T3_LB.setDefaultClamped(false);
	//
	Coordinate& T2_T3_AR = T2_T3_IVDjnt->upd_coordinates(2);
	T2_T3_AR.setDefaultLocked(false);
	T2_T3_AR.setDefaultClamped(false);
	//
	Coordinate& T2_T3_Tx = T2_T3_IVDjnt->upd_coordinates(3);
	T2_T3_Tx.setDefaultLocked(false);
	T2_T3_Tx.setDefaultClamped(true);
	//
	Coordinate& T2_T3_Ty = T2_T3_IVDjnt->upd_coordinates(4);
	T2_T3_Ty.setDefaultLocked(false);
	T2_T3_Ty.setDefaultClamped(true);
	//
	Coordinate& T2_T3_Tz = T2_T3_IVDjnt->upd_coordinates(5);
	T2_T3_Tz.setDefaultLocked(false);
	T2_T3_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_thoracic1_T1_T2_IVDjnt;
	st_thoracic1_T1_T2_IVDjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T1_T2_FE", 1, 1));
	st_thoracic1_T1_T2_IVDjnt[0].setFunction(new LinearFunction());
	st_thoracic1_T1_T2_IVDjnt[0].setAxis(Vec3(0, 0, 1));
	st_thoracic1_T1_T2_IVDjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T1_T2_LB", 1, 1));
	st_thoracic1_T1_T2_IVDjnt[1].setFunction(new LinearFunction());
	st_thoracic1_T1_T2_IVDjnt[1].setAxis(Vec3(1, 0, 0));
	st_thoracic1_T1_T2_IVDjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T1_T2_AR", 1, 1));
	st_thoracic1_T1_T2_IVDjnt[2].setFunction(new LinearFunction());
	st_thoracic1_T1_T2_IVDjnt[2].setAxis(Vec3(0, 1, 0));
	st_thoracic1_T1_T2_IVDjnt[3].setCoordinateNames(OpenSim::Array<std::string>("T1_T2_Tx", 1, 1));
	st_thoracic1_T1_T2_IVDjnt[3].setFunction(new LinearFunction());
	st_thoracic1_T1_T2_IVDjnt[3].setAxis(Vec3(1, 0, 0));
	st_thoracic1_T1_T2_IVDjnt[4].setCoordinateNames(OpenSim::Array<std::string>("T1_T2_Ty", 1, 1));
	st_thoracic1_T1_T2_IVDjnt[4].setFunction(new LinearFunction());
	st_thoracic1_T1_T2_IVDjnt[4].setAxis(Vec3(0, 1, 0));
	st_thoracic1_T1_T2_IVDjnt[5].setCoordinateNames(OpenSim::Array<std::string>("T1_T2_Tz", 1, 1));
	st_thoracic1_T1_T2_IVDjnt[5].setFunction(new LinearFunction());
	st_thoracic1_T1_T2_IVDjnt[5].setAxis(Vec3(0, 0, 1));
	T1_T2_IVDjnt = new CustomJoint("T1_T2_IVDjnt", *thoracic2, Vec3(0, 0.019107, 0), Vec3(0, 0, -0.017453), *thoracic1, Vec3(0, 0, 0), Vec3(0, 0, 0), st_thoracic1_T1_T2_IVDjnt);
	model->addJoint(T1_T2_IVDjnt);
	//
	//
	Coordinate& T1_T2_FE = T1_T2_IVDjnt->upd_coordinates(0);
	T1_T2_FE.setDefaultLocked(false);
	T1_T2_FE.setDefaultClamped(false);
	//
	Coordinate& T1_T2_LB = T1_T2_IVDjnt->upd_coordinates(1);
	T1_T2_LB.setDefaultLocked(false);
	T1_T2_LB.setDefaultClamped(false);
	//
	Coordinate& T1_T2_AR = T1_T2_IVDjnt->upd_coordinates(2);
	T1_T2_AR.setDefaultLocked(false);
	T1_T2_AR.setDefaultClamped(false);
	//
	Coordinate& T1_T2_Tx = T1_T2_IVDjnt->upd_coordinates(3);
	T1_T2_Tx.setDefaultLocked(false);
	T1_T2_Tx.setDefaultClamped(true);
	//
	Coordinate& T1_T2_Ty = T1_T2_IVDjnt->upd_coordinates(4);
	T1_T2_Ty.setDefaultLocked(false);
	T1_T2_Ty.setDefaultClamped(true);
	//
	Coordinate& T1_T2_Tz = T1_T2_IVDjnt->upd_coordinates(5);
	T1_T2_Tz.setDefaultLocked(false);
	T1_T2_Tz.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_cerv7_auxt1jnt;
	st_cerv7_auxt1jnt[0].setCoordinateNames(OpenSim::Array<std::string>("Neck_FE", 1, 1));
	st_cerv7_auxt1jnt[0].setFunction(new LinearFunction());
	st_cerv7_auxt1jnt[0].setAxis(Vec3(0, 0, 1));
	st_cerv7_auxt1jnt[1].setCoordinateNames(OpenSim::Array<std::string>("Neck_LB", 1, 1));
	st_cerv7_auxt1jnt[1].setFunction(new LinearFunction());
	st_cerv7_auxt1jnt[1].setAxis(Vec3(1, 0, 0));
	st_cerv7_auxt1jnt[2].setCoordinateNames(OpenSim::Array<std::string>("Neck_AR", 1, 1));
	st_cerv7_auxt1jnt[2].setFunction(new LinearFunction());
	st_cerv7_auxt1jnt[2].setAxis(Vec3(0, 1, 0));
	auxt1jnt = new CustomJoint("auxt1jnt", *thoracic1, Vec3(-0.0064474, 0.019107, 0), Vec3(0, 0, 0.5236), *cerv7, Vec3(0, 0, 0), Vec3(0, 0, 0), st_cerv7_auxt1jnt);
	model->addJoint(auxt1jnt);
	//
	//
	Coordinate& Neck_FE = auxt1jnt->upd_coordinates(0);
	Neck_FE.setDefaultLocked(false);
	Neck_FE.setDefaultClamped(true);
	//
	Coordinate& Neck_LB = auxt1jnt->upd_coordinates(1);
	Neck_LB.setDefaultLocked(false);
	Neck_LB.setDefaultClamped(true);
	//
	Coordinate& Neck_AR = auxt1jnt->upd_coordinates(2);
	Neck_AR.setDefaultLocked(false);
	Neck_AR.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_cerv6_aux7jnt;
	st_cerv6_aux7jnt[0].setCoordinateNames(OpenSim::Array<std::string>("aux7jnt_r3", 1, 1));
	st_cerv6_aux7jnt[0].setFunction(new LinearFunction());
	st_cerv6_aux7jnt[0].setAxis(Vec3(0, 0, 1));
	st_cerv6_aux7jnt[1].setCoordinateNames(OpenSim::Array<std::string>("aux7jnt_r1", 1, 1));
	st_cerv6_aux7jnt[1].setFunction(new LinearFunction());
	st_cerv6_aux7jnt[1].setAxis(Vec3(1, 0, 0));
	st_cerv6_aux7jnt[2].setCoordinateNames(OpenSim::Array<std::string>("aux7jnt_r2", 1, 1));
	st_cerv6_aux7jnt[2].setFunction(new LinearFunction());
	st_cerv6_aux7jnt[2].setAxis(Vec3(0, 1, 0));
	aux7jnt = new CustomJoint("aux7jnt", *cerv7, Vec3(0.012495, 0.011669, 0), Vec3(0, 0, 0), *cerv6, Vec3(0.0088784, -0.0038317, 0), Vec3(0, 0, 0), st_cerv6_aux7jnt);
	model->addJoint(aux7jnt);
	//
	//
	Coordinate& aux7jnt_r3 = aux7jnt->upd_coordinates(0);
	aux7jnt_r3.setDefaultLocked(true);
	aux7jnt_r3.setDefaultClamped(false);
	//
	Coordinate& aux7jnt_r1 = aux7jnt->upd_coordinates(1);
	aux7jnt_r1.setDefaultLocked(true);
	aux7jnt_r1.setDefaultClamped(false);
	//
	Coordinate& aux7jnt_r2 = aux7jnt->upd_coordinates(2);
	aux7jnt_r2.setDefaultLocked(true);
	aux7jnt_r2.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_cerv5_aux6jnt;
	st_cerv5_aux6jnt[0].setCoordinateNames(OpenSim::Array<std::string>("aux6jnt_r3", 1, 1));
	st_cerv5_aux6jnt[0].setFunction(new LinearFunction());
	st_cerv5_aux6jnt[0].setAxis(Vec3(0, 0, 1));
	st_cerv5_aux6jnt[1].setCoordinateNames(OpenSim::Array<std::string>("aux6jnt_r1", 1, 1));
	st_cerv5_aux6jnt[1].setFunction(new LinearFunction());
	st_cerv5_aux6jnt[1].setAxis(Vec3(1, 0, 0));
	st_cerv5_aux6jnt[2].setCoordinateNames(OpenSim::Array<std::string>("aux6jnt_r2", 1, 1));
	st_cerv5_aux6jnt[2].setFunction(new LinearFunction());
	st_cerv5_aux6jnt[2].setAxis(Vec3(0, 1, 0));
	aux6jnt = new CustomJoint("aux6jnt", *cerv6, Vec3(0.0092846, 0.0091904, 0), Vec3(0, 0, 0), *cerv5, Vec3(0.0053445, -0.0076822, 0), Vec3(0, 0, 0), st_cerv5_aux6jnt);
	model->addJoint(aux6jnt);
	//
	//
	Coordinate& aux6jnt_r3 = aux6jnt->upd_coordinates(0);
	aux6jnt_r3.setDefaultLocked(true);
	aux6jnt_r3.setDefaultClamped(false);
	//
	Coordinate& aux6jnt_r1 = aux6jnt->upd_coordinates(1);
	aux6jnt_r1.setDefaultLocked(true);
	aux6jnt_r1.setDefaultClamped(false);
	//
	Coordinate& aux6jnt_r2 = aux6jnt->upd_coordinates(2);
	aux6jnt_r2.setDefaultLocked(true);
	aux6jnt_r2.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_cerv4_aux5jnt;
	st_cerv4_aux5jnt[0].setCoordinateNames(OpenSim::Array<std::string>("aux5jnt_r3", 1, 1));
	st_cerv4_aux5jnt[0].setFunction(new LinearFunction());
	st_cerv4_aux5jnt[0].setAxis(Vec3(0, 0, 1));
	st_cerv4_aux5jnt[1].setCoordinateNames(OpenSim::Array<std::string>("aux5jnt_r1", 1, 1));
	st_cerv4_aux5jnt[1].setFunction(new LinearFunction());
	st_cerv4_aux5jnt[1].setAxis(Vec3(1, 0, 0));
	st_cerv4_aux5jnt[2].setCoordinateNames(OpenSim::Array<std::string>("aux5jnt_r2", 1, 1));
	st_cerv4_aux5jnt[2].setFunction(new LinearFunction());
	st_cerv4_aux5jnt[2].setAxis(Vec3(0, 1, 0));
	aux5jnt = new CustomJoint("aux5jnt", *cerv5, Vec3(0.0079461, 0.0070224, 0), Vec3(0, 0, 0), *cerv4, Vec3(0.0047319, -0.011047, 0), Vec3(0, 0, 0), st_cerv4_aux5jnt);
	model->addJoint(aux5jnt);
	//
	//
	Coordinate& aux5jnt_r3 = aux5jnt->upd_coordinates(0);
	aux5jnt_r3.setDefaultLocked(true);
	aux5jnt_r3.setDefaultClamped(false);
	//
	Coordinate& aux5jnt_r1 = aux5jnt->upd_coordinates(1);
	aux5jnt_r1.setDefaultLocked(true);
	aux5jnt_r1.setDefaultClamped(false);
	//
	Coordinate& aux5jnt_r2 = aux5jnt->upd_coordinates(2);
	aux5jnt_r2.setDefaultLocked(true);
	aux5jnt_r2.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_cerv3_aux4jnt;
	st_cerv3_aux4jnt[0].setCoordinateNames(OpenSim::Array<std::string>("aux4jnt_r3", 1, 1));
	st_cerv3_aux4jnt[0].setFunction(new LinearFunction());
	st_cerv3_aux4jnt[0].setAxis(Vec3(0, 0, 1));
	st_cerv3_aux4jnt[1].setCoordinateNames(OpenSim::Array<std::string>("aux4jnt_r1", 1, 1));
	st_cerv3_aux4jnt[1].setFunction(new LinearFunction());
	st_cerv3_aux4jnt[1].setAxis(Vec3(1, 0, 0));
	st_cerv3_aux4jnt[2].setCoordinateNames(OpenSim::Array<std::string>("aux4jnt_r2", 1, 1));
	st_cerv3_aux4jnt[2].setFunction(new LinearFunction());
	st_cerv3_aux4jnt[2].setAxis(Vec3(0, 1, 0));
	aux4jnt = new CustomJoint("aux4jnt", *cerv4, Vec3(0.0068998, 0.0060138, 0), Vec3(0, 0, 0), *cerv3, Vec3(0.0044396, -0.0085023, 0), Vec3(0, 0, 0), st_cerv3_aux4jnt);
	model->addJoint(aux4jnt);
	//
	//
	Coordinate& aux4jnt_r3 = aux4jnt->upd_coordinates(0);
	aux4jnt_r3.setDefaultLocked(true);
	aux4jnt_r3.setDefaultClamped(false);
	//
	Coordinate& aux4jnt_r1 = aux4jnt->upd_coordinates(1);
	aux4jnt_r1.setDefaultLocked(true);
	aux4jnt_r1.setDefaultClamped(false);
	//
	Coordinate& aux4jnt_r2 = aux4jnt->upd_coordinates(2);
	aux4jnt_r2.setDefaultLocked(true);
	aux4jnt_r2.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_cerv2_aux3jnt;
	st_cerv2_aux3jnt[0].setCoordinateNames(OpenSim::Array<std::string>("aux3jnt_r3", 1, 1));
	st_cerv2_aux3jnt[0].setFunction(new LinearFunction());
	st_cerv2_aux3jnt[0].setAxis(Vec3(0, 0, 1));
	st_cerv2_aux3jnt[1].setCoordinateNames(OpenSim::Array<std::string>("aux3jnt_r1", 1, 1));
	st_cerv2_aux3jnt[1].setFunction(new LinearFunction());
	st_cerv2_aux3jnt[1].setAxis(Vec3(1, 0, 0));
	st_cerv2_aux3jnt[2].setCoordinateNames(OpenSim::Array<std::string>("aux3jnt_r2", 1, 1));
	st_cerv2_aux3jnt[2].setFunction(new LinearFunction());
	st_cerv2_aux3jnt[2].setAxis(Vec3(0, 1, 0));
	aux3jnt = new CustomJoint("aux3jnt", *cerv3, Vec3(0.0049487, 0.0047696, 0), Vec3(0, 0, 0), *cerv2, Vec3(0.0051183, -0.010585, 0), Vec3(0, 0, 0), st_cerv2_aux3jnt);
	model->addJoint(aux3jnt);
	//
	//
	Coordinate& aux3jnt_r3 = aux3jnt->upd_coordinates(0);
	aux3jnt_r3.setDefaultLocked(true);
	aux3jnt_r3.setDefaultClamped(false);
	//
	Coordinate& aux3jnt_r1 = aux3jnt->upd_coordinates(1);
	aux3jnt_r1.setDefaultLocked(true);
	aux3jnt_r1.setDefaultClamped(false);
	//
	Coordinate& aux3jnt_r2 = aux3jnt->upd_coordinates(2);
	aux3jnt_r2.setDefaultLocked(true);
	aux3jnt_r2.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_cerv1_aux2jnt;
	st_cerv1_aux2jnt[0].setCoordinateNames(OpenSim::Array<std::string>("Head_FE", 1, 1));
	st_cerv1_aux2jnt[0].setFunction(new LinearFunction());
	st_cerv1_aux2jnt[0].setAxis(Vec3(0, 0, 1));
	st_cerv1_aux2jnt[1].setCoordinateNames(OpenSim::Array<std::string>("Head_LB", 1, 1));
	st_cerv1_aux2jnt[1].setFunction(new LinearFunction());
	st_cerv1_aux2jnt[1].setAxis(Vec3(1, 0, 0));
	st_cerv1_aux2jnt[2].setCoordinateNames(OpenSim::Array<std::string>("Head_AR", 1, 1));
	st_cerv1_aux2jnt[2].setFunction(new LinearFunction());
	st_cerv1_aux2jnt[2].setAxis(Vec3(0, 1, 0));
	aux2jnt = new CustomJoint("aux2jnt", *cerv2, Vec3(0.0064945, 0.020059, 0), Vec3(0, 0, 0), *cerv1, Vec3(0.040013, 0.003318, 0), Vec3(0, 0, 0), st_cerv1_aux2jnt);
	model->addJoint(aux2jnt);
	//
	//
	Coordinate& Head_FE = aux2jnt->upd_coordinates(0);
	Head_FE.setDefaultLocked(false);
	Head_FE.setDefaultClamped(true);
	//
	Coordinate& Head_LB = aux2jnt->upd_coordinates(1);
	Head_LB.setDefaultLocked(false);
	Head_LB.setDefaultClamped(true);
	//
	Coordinate& Head_AR = aux2jnt->upd_coordinates(2);
	Head_AR.setDefaultLocked(false);
	Head_AR.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_skull_aux1jnt;
	st_skull_aux1jnt[0].setCoordinateNames(OpenSim::Array<std::string>("aux1jnt_r3", 1, 1));
	st_skull_aux1jnt[0].setFunction(new LinearFunction());
	st_skull_aux1jnt[0].setAxis(Vec3(0, 0, 1));
	st_skull_aux1jnt[1].setCoordinateNames(OpenSim::Array<std::string>("aux1jnt_r1", 1, 1));
	st_skull_aux1jnt[1].setFunction(new LinearFunction());
	st_skull_aux1jnt[1].setAxis(Vec3(1, 0, 0));
	st_skull_aux1jnt[2].setCoordinateNames(OpenSim::Array<std::string>("aux1jnt_r2", 1, 1));
	st_skull_aux1jnt[2].setFunction(new LinearFunction());
	st_skull_aux1jnt[2].setAxis(Vec3(0, 1, 0));
	aux1jnt = new CustomJoint("aux1jnt", *cerv1, Vec3(0.040862, 0.014064, 0), Vec3(0, 0, 0), *skull, Vec3(0, 0, 0), Vec3(0, 0, 0), st_skull_aux1jnt);
	model->addJoint(aux1jnt);
	//
	//
	Coordinate& aux1jnt_r3 = aux1jnt->upd_coordinates(0);
	aux1jnt_r3.setDefaultLocked(true);
	aux1jnt_r3.setDefaultClamped(false);
	//
	Coordinate& aux1jnt_r1 = aux1jnt->upd_coordinates(1);
	aux1jnt_r1.setDefaultLocked(true);
	aux1jnt_r1.setDefaultClamped(false);
	//
	Coordinate& aux1jnt_r2 = aux1jnt->upd_coordinates(2);
	aux1jnt_r2.setDefaultLocked(true);
	aux1jnt_r2.setDefaultClamped(false);
	model->updJointSet();
	//
	jawjnt_weld = new WeldJoint("jawjnt_weld", *skull, Vec3(0, 0, 0), Vec3(0, 0, 0), *jaw, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(jawjnt_weld);
	//
	SpatialTransform st_rib12_R_T12_r12R_CVjnt;
	st_rib12_R_T12_r12R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T12_r12R_X", 1, 1));
	st_rib12_R_T12_r12R_CVjnt[0].setFunction(new LinearFunction());
	st_rib12_R_T12_r12R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib12_R_T12_r12R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T12_r12R_Y", 1, 1));
	st_rib12_R_T12_r12R_CVjnt[1].setFunction(new LinearFunction());
	st_rib12_R_T12_r12R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib12_R_T12_r12R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T12_r12R_Z", 1, 1));
	st_rib12_R_T12_r12R_CVjnt[2].setFunction(new LinearFunction());
	st_rib12_R_T12_r12R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T12_r12R_CVjnt = new CustomJoint("T12_r12R_CVjnt", *thoracic12, Vec3(-0.021649, 0.021661, 0.021037), Vec3(1.5708, 3.1416, 0.2748), *rib12_R, Vec3(0, 0, 0), Vec3(-1.5708, -0.34907, -2.8668), st_rib12_R_T12_r12R_CVjnt);
	model->addJoint(T12_r12R_CVjnt);
	//
	//
	Coordinate& T12_r12R_X = T12_r12R_CVjnt->upd_coordinates(0);
	T12_r12R_X.setDefaultLocked(true);
	T12_r12R_X.setDefaultClamped(false);
	//
	Coordinate& T12_r12R_Y = T12_r12R_CVjnt->upd_coordinates(1);
	T12_r12R_Y.setDefaultLocked(true);
	T12_r12R_Y.setDefaultClamped(false);
	//
	Coordinate& T12_r12R_Z = T12_r12R_CVjnt->upd_coordinates(2);
	T12_r12R_Z.setDefaultLocked(true);
	T12_r12R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib11_R_T11_r11R_CVjnt;
	st_rib11_R_T11_r11R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T11_r11R_X", 1, 1));
	st_rib11_R_T11_r11R_CVjnt[0].setFunction(new LinearFunction());
	st_rib11_R_T11_r11R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib11_R_T11_r11R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T11_r11R_Y", 1, 1));
	st_rib11_R_T11_r11R_CVjnt[1].setFunction(new LinearFunction());
	st_rib11_R_T11_r11R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib11_R_T11_r11R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T11_r11R_Z", 1, 1));
	st_rib11_R_T11_r11R_CVjnt[2].setFunction(new LinearFunction());
	st_rib11_R_T11_r11R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T11_r11R_CVjnt = new CustomJoint("T11_r11R_CVjnt", *thoracic11, Vec3(-0.015692, 0.018461, 0.016487), Vec3(1.5708, 3.1416, 0.34175), *rib11_R, Vec3(0, 0, 0), Vec3(-1.5708, -0.27925, -2.7998), st_rib11_R_T11_r11R_CVjnt);
	model->addJoint(T11_r11R_CVjnt);
	//
	//
	Coordinate& T11_r11R_X = T11_r11R_CVjnt->upd_coordinates(0);
	T11_r11R_X.setDefaultLocked(true);
	T11_r11R_X.setDefaultClamped(false);
	//
	Coordinate& T11_r11R_Y = T11_r11R_CVjnt->upd_coordinates(1);
	T11_r11R_Y.setDefaultLocked(true);
	T11_r11R_Y.setDefaultClamped(false);
	//
	Coordinate& T11_r11R_Z = T11_r11R_CVjnt->upd_coordinates(2);
	T11_r11R_Z.setDefaultLocked(true);
	T11_r11R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib10_R_T10_r10R_CVjnt;
	st_rib10_R_T10_r10R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T10_r10R_X", 1, 1));
	st_rib10_R_T10_r10R_CVjnt[0].setFunction(new LinearFunction());
	st_rib10_R_T10_r10R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib10_R_T10_r10R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T10_r10R_Y", 1, 1));
	st_rib10_R_T10_r10R_CVjnt[1].setFunction(new LinearFunction());
	st_rib10_R_T10_r10R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib10_R_T10_r10R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T10_r10R_Z", 1, 1));
	st_rib10_R_T10_r10R_CVjnt[2].setFunction(new LinearFunction());
	st_rib10_R_T10_r10R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T10_r10R_CVjnt = new CustomJoint("T10_r10R_CVjnt", *thoracic10, Vec3(-0.014615, 0.028477, 0.016713), Vec3(1.5708, 3.1416, 0.40034), *rib10_R, Vec3(0, 0, 0), Vec3(-1.5708, -0.19199, -2.7413), st_rib10_R_T10_r10R_CVjnt);
	model->addJoint(T10_r10R_CVjnt);
	//
	//
	Coordinate& T10_r10R_X = T10_r10R_CVjnt->upd_coordinates(0);
	T10_r10R_X.setDefaultLocked(true);
	T10_r10R_X.setDefaultClamped(false);
	//
	Coordinate& T10_r10R_Y = T10_r10R_CVjnt->upd_coordinates(1);
	T10_r10R_Y.setDefaultLocked(true);
	T10_r10R_Y.setDefaultClamped(false);
	//
	Coordinate& T10_r10R_Z = T10_r10R_CVjnt->upd_coordinates(2);
	T10_r10R_Z.setDefaultLocked(true);
	T10_r10R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib9_R_T9_r9R_CVjnt;
	st_rib9_R_T9_r9R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T9_r9R_X", 1, 1));
	st_rib9_R_T9_r9R_CVjnt[0].setFunction(new LinearFunction());
	st_rib9_R_T9_r9R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib9_R_T9_r9R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T9_r9R_Y", 1, 1));
	st_rib9_R_T9_r9R_CVjnt[1].setFunction(new LinearFunction());
	st_rib9_R_T9_r9R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib9_R_T9_r9R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T9_r9R_Z", 1, 1));
	st_rib9_R_T9_r9R_CVjnt[2].setFunction(new LinearFunction());
	st_rib9_R_T9_r9R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T9_r9R_CVjnt = new CustomJoint("T9_r9R_CVjnt", *thoracic9, Vec3(-0.011492, 0.024682, 0.014417), Vec3(1.5708, 3.1416, 0.45172), *rib9_R, Vec3(0, 0, 0), Vec3(-1.5708, -0.12217, -2.6899), st_rib9_R_T9_r9R_CVjnt);
	model->addJoint(T9_r9R_CVjnt);
	//
	//
	Coordinate& T9_r9R_X = T9_r9R_CVjnt->upd_coordinates(0);
	T9_r9R_X.setDefaultLocked(true);
	T9_r9R_X.setDefaultClamped(false);
	//
	Coordinate& T9_r9R_Y = T9_r9R_CVjnt->upd_coordinates(1);
	T9_r9R_Y.setDefaultLocked(true);
	T9_r9R_Y.setDefaultClamped(false);
	//
	Coordinate& T9_r9R_Z = T9_r9R_CVjnt->upd_coordinates(2);
	T9_r9R_Z.setDefaultLocked(true);
	T9_r9R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib8_R_T8_r8R_CVjnt;
	st_rib8_R_T8_r8R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T8_r8R_X", 1, 1));
	st_rib8_R_T8_r8R_CVjnt[0].setFunction(new LinearFunction());
	st_rib8_R_T8_r8R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib8_R_T8_r8R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T8_r8R_Y", 1, 1));
	st_rib8_R_T8_r8R_CVjnt[1].setFunction(new LinearFunction());
	st_rib8_R_T8_r8R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib8_R_T8_r8R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T8_r8R_Z", 1, 1));
	st_rib8_R_T8_r8R_CVjnt[2].setFunction(new LinearFunction());
	st_rib8_R_T8_r8R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T8_r8R_CVjnt = new CustomJoint("T8_r8R_CVjnt", *thoracic8, Vec3(-0.010307, 0.024171, 0.014319), Vec3(1.5708, 3.1416, 0.49691), *rib8_R, Vec3(0, 0, 0), Vec3(-1.5708, -0.034907, -2.6447), st_rib8_R_T8_r8R_CVjnt);
	model->addJoint(T8_r8R_CVjnt);
	//
	//
	Coordinate& T8_r8R_X = T8_r8R_CVjnt->upd_coordinates(0);
	T8_r8R_X.setDefaultLocked(true);
	T8_r8R_X.setDefaultClamped(false);
	//
	Coordinate& T8_r8R_Y = T8_r8R_CVjnt->upd_coordinates(1);
	T8_r8R_Y.setDefaultLocked(true);
	T8_r8R_Y.setDefaultClamped(false);
	//
	Coordinate& T8_r8R_Z = T8_r8R_CVjnt->upd_coordinates(2);
	T8_r8R_Z.setDefaultLocked(true);
	T8_r8R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib7_R_T7_r7R_CVjnt;
	st_rib7_R_T7_r7R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T7_r7R_X", 1, 1));
	st_rib7_R_T7_r7R_CVjnt[0].setFunction(new LinearFunction());
	st_rib7_R_T7_r7R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib7_R_T7_r7R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T7_r7R_Y", 1, 1));
	st_rib7_R_T7_r7R_CVjnt[1].setFunction(new LinearFunction());
	st_rib7_R_T7_r7R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib7_R_T7_r7R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T7_r7R_Z", 1, 1));
	st_rib7_R_T7_r7R_CVjnt[2].setFunction(new LinearFunction());
	st_rib7_R_T7_r7R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T7_r7R_CVjnt = new CustomJoint("T7_r7R_CVjnt", *thoracic7, Vec3(-0.0098876, 0.025467, 0.01539), Vec3(1.5708, 3.1416, 0.53682), *rib7_R, Vec3(0, 0, 0), Vec3(-1.5708, 0.069813, -2.6048), st_rib7_R_T7_r7R_CVjnt);
	model->addJoint(T7_r7R_CVjnt);
	//
	//
	Coordinate& T7_r7R_X = T7_r7R_CVjnt->upd_coordinates(0);
	T7_r7R_X.setDefaultLocked(true);
	T7_r7R_X.setDefaultClamped(false);
	//
	Coordinate& T7_r7R_Y = T7_r7R_CVjnt->upd_coordinates(1);
	T7_r7R_Y.setDefaultLocked(true);
	T7_r7R_Y.setDefaultClamped(false);
	//
	Coordinate& T7_r7R_Z = T7_r7R_CVjnt->upd_coordinates(2);
	T7_r7R_Z.setDefaultLocked(true);
	T7_r7R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib6_R_T6_r6R_CVjnt;
	st_rib6_R_T6_r6R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T6_r6R_X", 1, 1));
	st_rib6_R_T6_r6R_CVjnt[0].setFunction(new LinearFunction());
	st_rib6_R_T6_r6R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib6_R_T6_r6R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T6_r6R_Y", 1, 1));
	st_rib6_R_T6_r6R_CVjnt[1].setFunction(new LinearFunction());
	st_rib6_R_T6_r6R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib6_R_T6_r6R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T6_r6R_Z", 1, 1));
	st_rib6_R_T6_r6R_CVjnt[2].setFunction(new LinearFunction());
	st_rib6_R_T6_r6R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T6_r6R_CVjnt = new CustomJoint("T6_r6R_CVjnt", *thoracic6, Vec3(-0.0083438, 0.024198, 0.014764), Vec3(1.5708, 3.1416, 0.57221), *rib6_R, Vec3(0, 0, 0), Vec3(-1.5708, 0.17453, -2.5694), st_rib6_R_T6_r6R_CVjnt);
	model->addJoint(T6_r6R_CVjnt);
	//
	//
	Coordinate& T6_r6R_X = T6_r6R_CVjnt->upd_coordinates(0);
	T6_r6R_X.setDefaultLocked(true);
	T6_r6R_X.setDefaultClamped(false);
	//
	Coordinate& T6_r6R_Y = T6_r6R_CVjnt->upd_coordinates(1);
	T6_r6R_Y.setDefaultLocked(true);
	T6_r6R_Y.setDefaultClamped(false);
	//
	Coordinate& T6_r6R_Z = T6_r6R_CVjnt->upd_coordinates(2);
	T6_r6R_Z.setDefaultLocked(true);
	T6_r6R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib5_R_T5_r5R_CVjnt;
	st_rib5_R_T5_r5R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T5_r5R_X", 1, 1));
	st_rib5_R_T5_r5R_CVjnt[0].setFunction(new LinearFunction());
	st_rib5_R_T5_r5R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib5_R_T5_r5R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T5_r5R_Y", 1, 1));
	st_rib5_R_T5_r5R_CVjnt[1].setFunction(new LinearFunction());
	st_rib5_R_T5_r5R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib5_R_T5_r5R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T5_r5R_Z", 1, 1));
	st_rib5_R_T5_r5R_CVjnt[2].setFunction(new LinearFunction());
	st_rib5_R_T5_r5R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T5_r5R_CVjnt = new CustomJoint("T5_r5R_CVjnt", *thoracic5, Vec3(-0.0074323, 0.024759, 0.015236), Vec3(1.5708, 3.1416, 0.60373), *rib5_R, Vec3(0, 0, 0), Vec3(-1.5708, 0.27925, -2.5379), st_rib5_R_T5_r5R_CVjnt);
	model->addJoint(T5_r5R_CVjnt);
	//
	//
	Coordinate& T5_r5R_X = T5_r5R_CVjnt->upd_coordinates(0);
	T5_r5R_X.setDefaultLocked(true);
	T5_r5R_X.setDefaultClamped(false);
	//
	Coordinate& T5_r5R_Y = T5_r5R_CVjnt->upd_coordinates(1);
	T5_r5R_Y.setDefaultLocked(true);
	T5_r5R_Y.setDefaultClamped(false);
	//
	Coordinate& T5_r5R_Z = T5_r5R_CVjnt->upd_coordinates(2);
	T5_r5R_Z.setDefaultLocked(true);
	T5_r5R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib4_R_T4_r4R_CVjnt;
	st_rib4_R_T4_r4R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T4_r4R_X", 1, 1));
	st_rib4_R_T4_r4R_CVjnt[0].setFunction(new LinearFunction());
	st_rib4_R_T4_r4R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib4_R_T4_r4R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T4_r4R_Y", 1, 1));
	st_rib4_R_T4_r4R_CVjnt[1].setFunction(new LinearFunction());
	st_rib4_R_T4_r4R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib4_R_T4_r4R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T4_r4R_Z", 1, 1));
	st_rib4_R_T4_r4R_CVjnt[2].setFunction(new LinearFunction());
	st_rib4_R_T4_r4R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T4_r4R_CVjnt = new CustomJoint("T4_r4R_CVjnt", *thoracic4, Vec3(-0.0057726, 0.02307, 0.014063), Vec3(1.5708, 3.1416, 0.63193), *rib4_R, Vec3(0, 0, 0), Vec3(-1.5708, 0.38397, -2.5097), st_rib4_R_T4_r4R_CVjnt);
	model->addJoint(T4_r4R_CVjnt);
	//
	//
	Coordinate& T4_r4R_X = T4_r4R_CVjnt->upd_coordinates(0);
	T4_r4R_X.setDefaultLocked(true);
	T4_r4R_X.setDefaultClamped(false);
	//
	Coordinate& T4_r4R_Y = T4_r4R_CVjnt->upd_coordinates(1);
	T4_r4R_Y.setDefaultLocked(true);
	T4_r4R_Y.setDefaultClamped(false);
	//
	Coordinate& T4_r4R_Z = T4_r4R_CVjnt->upd_coordinates(2);
	T4_r4R_Z.setDefaultLocked(true);
	T4_r4R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib3_R_T3_r3R_CVjnt;
	st_rib3_R_T3_r3R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T3_r3R_X", 1, 1));
	st_rib3_R_T3_r3R_CVjnt[0].setFunction(new LinearFunction());
	st_rib3_R_T3_r3R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib3_R_T3_r3R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T3_r3R_Y", 1, 1));
	st_rib3_R_T3_r3R_CVjnt[1].setFunction(new LinearFunction());
	st_rib3_R_T3_r3R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib3_R_T3_r3R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T3_r3R_Z", 1, 1));
	st_rib3_R_T3_r3R_CVjnt[2].setFunction(new LinearFunction());
	st_rib3_R_T3_r3R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T3_r3R_CVjnt = new CustomJoint("T3_r3R_CVjnt", *thoracic3, Vec3(-0.004191, 0.020935, 0.012579), Vec3(1.5708, 3.1416, 0.65727), *rib3_R, Vec3(0, 0, 0), Vec3(-1.5708, 0.45379, -2.4843), st_rib3_R_T3_r3R_CVjnt);
	model->addJoint(T3_r3R_CVjnt);
	//
	//
	Coordinate& T3_r3R_X = T3_r3R_CVjnt->upd_coordinates(0);
	T3_r3R_X.setDefaultLocked(true);
	T3_r3R_X.setDefaultClamped(false);
	//
	Coordinate& T3_r3R_Y = T3_r3R_CVjnt->upd_coordinates(1);
	T3_r3R_Y.setDefaultLocked(true);
	T3_r3R_Y.setDefaultClamped(false);
	//
	Coordinate& T3_r3R_Z = T3_r3R_CVjnt->upd_coordinates(2);
	T3_r3R_Z.setDefaultLocked(true);
	T3_r3R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib2_R_T2_r2R_CVjnt;
	st_rib2_R_T2_r2R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T2_r2R_X", 1, 1));
	st_rib2_R_T2_r2R_CVjnt[0].setFunction(new LinearFunction());
	st_rib2_R_T2_r2R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib2_R_T2_r2R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T2_r2R_Y", 1, 1));
	st_rib2_R_T2_r2R_CVjnt[1].setFunction(new LinearFunction());
	st_rib2_R_T2_r2R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib2_R_T2_r2R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T2_r2R_Z", 1, 1));
	st_rib2_R_T2_r2R_CVjnt[2].setFunction(new LinearFunction());
	st_rib2_R_T2_r2R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T2_r2R_CVjnt = new CustomJoint("T2_r2R_CVjnt", *thoracic2, Vec3(-0.0028935, 0.019107, 0.011309), Vec3(1.5708, 3.1416, 0.68013), *rib2_R, Vec3(0, 0, 0), Vec3(-1.5708, 0.50615, -2.4615), st_rib2_R_T2_r2R_CVjnt);
	model->addJoint(T2_r2R_CVjnt);
	//
	//
	Coordinate& T2_r2R_X = T2_r2R_CVjnt->upd_coordinates(0);
	T2_r2R_X.setDefaultLocked(true);
	T2_r2R_X.setDefaultClamped(false);
	//
	Coordinate& T2_r2R_Y = T2_r2R_CVjnt->upd_coordinates(1);
	T2_r2R_Y.setDefaultLocked(true);
	T2_r2R_Y.setDefaultClamped(false);
	//
	Coordinate& T2_r2R_Z = T2_r2R_CVjnt->upd_coordinates(2);
	T2_r2R_Z.setDefaultLocked(true);
	T2_r2R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib1_R_T1_r1R_CVjnt;
	st_rib1_R_T1_r1R_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T1_r1R_X", 1, 1));
	st_rib1_R_T1_r1R_CVjnt[0].setFunction(new LinearFunction());
	st_rib1_R_T1_r1R_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib1_R_T1_r1R_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T1_r1R_Y", 1, 1));
	st_rib1_R_T1_r1R_CVjnt[1].setFunction(new LinearFunction());
	st_rib1_R_T1_r1R_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib1_R_T1_r1R_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T1_r1R_Z", 1, 1));
	st_rib1_R_T1_r1R_CVjnt[2].setFunction(new LinearFunction());
	st_rib1_R_T1_r1R_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T1_r1R_CVjnt = new CustomJoint("T1_r1R_CVjnt", *thoracic1, Vec3(-0.0031561, 0.018379, 0.017678), Vec3(1.5708, 3.1416, 0.70085), *rib1_R, Vec3(0, 0, 0), Vec3(-1.5708, 0.5236, -2.4407), st_rib1_R_T1_r1R_CVjnt);
	model->addJoint(T1_r1R_CVjnt);
	//
	//
	Coordinate& T1_r1R_X = T1_r1R_CVjnt->upd_coordinates(0);
	T1_r1R_X.setDefaultLocked(true);
	T1_r1R_X.setDefaultClamped(false);
	//
	Coordinate& T1_r1R_Y = T1_r1R_CVjnt->upd_coordinates(1);
	T1_r1R_Y.setDefaultLocked(true);
	T1_r1R_Y.setDefaultClamped(false);
	//
	Coordinate& T1_r1R_Z = T1_r1R_CVjnt->upd_coordinates(2);
	T1_r1R_Z.setDefaultLocked(true);
	T1_r1R_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib12_L_T12_r12L_CVjnt;
	st_rib12_L_T12_r12L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T12_r12L_X", 1, 1));
	st_rib12_L_T12_r12L_CVjnt[0].setFunction(new LinearFunction());
	st_rib12_L_T12_r12L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib12_L_T12_r12L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T12_r12L_Y", 1, 1));
	st_rib12_L_T12_r12L_CVjnt[1].setFunction(new LinearFunction());
	st_rib12_L_T12_r12L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib12_L_T12_r12L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T12_r12L_Z", 1, 1));
	st_rib12_L_T12_r12L_CVjnt[2].setFunction(new LinearFunction());
	st_rib12_L_T12_r12L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T12_r12L_CVjnt = new CustomJoint("T12_r12L_CVjnt", *thoracic12, Vec3(-0.021649, 0.021661, -0.021037), Vec3(1.5708, 3.1416, -0.2748), *rib12_L, Vec3(0, 0, 0), Vec3(-1.5708, -0.34907, 2.8668), st_rib12_L_T12_r12L_CVjnt);
	model->addJoint(T12_r12L_CVjnt);
	//
	//
	Coordinate& T12_r12L_X = T12_r12L_CVjnt->upd_coordinates(0);
	T12_r12L_X.setDefaultLocked(true);
	T12_r12L_X.setDefaultClamped(false);
	//
	Coordinate& T12_r12L_Y = T12_r12L_CVjnt->upd_coordinates(1);
	T12_r12L_Y.setDefaultLocked(true);
	T12_r12L_Y.setDefaultClamped(false);
	//
	Coordinate& T12_r12L_Z = T12_r12L_CVjnt->upd_coordinates(2);
	T12_r12L_Z.setDefaultLocked(true);
	T12_r12L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib11_L_T11_r11L_CVjnt;
	st_rib11_L_T11_r11L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T11_r11L_X", 1, 1));
	st_rib11_L_T11_r11L_CVjnt[0].setFunction(new LinearFunction());
	st_rib11_L_T11_r11L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib11_L_T11_r11L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T11_r11L_Y", 1, 1));
	st_rib11_L_T11_r11L_CVjnt[1].setFunction(new LinearFunction());
	st_rib11_L_T11_r11L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib11_L_T11_r11L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T11_r11L_Z", 1, 1));
	st_rib11_L_T11_r11L_CVjnt[2].setFunction(new LinearFunction());
	st_rib11_L_T11_r11L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T11_r11L_CVjnt = new CustomJoint("T11_r11L_CVjnt", *thoracic11, Vec3(-0.015692, 0.018461, -0.016487), Vec3(1.5708, 3.1416, -0.34175), *rib11_L, Vec3(0, 0, 0), Vec3(-1.5708, -0.27925, 2.7998), st_rib11_L_T11_r11L_CVjnt);
	model->addJoint(T11_r11L_CVjnt);
	//
	//
	Coordinate& T11_r11L_X = T11_r11L_CVjnt->upd_coordinates(0);
	T11_r11L_X.setDefaultLocked(true);
	T11_r11L_X.setDefaultClamped(false);
	//
	Coordinate& T11_r11L_Y = T11_r11L_CVjnt->upd_coordinates(1);
	T11_r11L_Y.setDefaultLocked(true);
	T11_r11L_Y.setDefaultClamped(false);
	//
	Coordinate& T11_r11L_Z = T11_r11L_CVjnt->upd_coordinates(2);
	T11_r11L_Z.setDefaultLocked(true);
	T11_r11L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib10_L_T10_r10L_CVjnt;
	st_rib10_L_T10_r10L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T10_r10L_X", 1, 1));
	st_rib10_L_T10_r10L_CVjnt[0].setFunction(new LinearFunction());
	st_rib10_L_T10_r10L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib10_L_T10_r10L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T10_r10L_Y", 1, 1));
	st_rib10_L_T10_r10L_CVjnt[1].setFunction(new LinearFunction());
	st_rib10_L_T10_r10L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib10_L_T10_r10L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T10_r10L_Z", 1, 1));
	st_rib10_L_T10_r10L_CVjnt[2].setFunction(new LinearFunction());
	st_rib10_L_T10_r10L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T10_r10L_CVjnt = new CustomJoint("T10_r10L_CVjnt", *thoracic10, Vec3(-0.014615, 0.028477, -0.016713), Vec3(1.5708, 3.1416, -0.40034), *rib10_L, Vec3(0, 0, 0), Vec3(-1.5708, -0.19199, 2.7413), st_rib10_L_T10_r10L_CVjnt);
	model->addJoint(T10_r10L_CVjnt);
	//
	//
	Coordinate& T10_r10L_X = T10_r10L_CVjnt->upd_coordinates(0);
	T10_r10L_X.setDefaultLocked(true);
	T10_r10L_X.setDefaultClamped(false);
	//
	Coordinate& T10_r10L_Y = T10_r10L_CVjnt->upd_coordinates(1);
	T10_r10L_Y.setDefaultLocked(true);
	T10_r10L_Y.setDefaultClamped(false);
	//
	Coordinate& T10_r10L_Z = T10_r10L_CVjnt->upd_coordinates(2);
	T10_r10L_Z.setDefaultLocked(true);
	T10_r10L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib9_L_T9_r9L_CVjnt;
	st_rib9_L_T9_r9L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T9_r9L_X", 1, 1));
	st_rib9_L_T9_r9L_CVjnt[0].setFunction(new LinearFunction());
	st_rib9_L_T9_r9L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib9_L_T9_r9L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T9_r9L_Y", 1, 1));
	st_rib9_L_T9_r9L_CVjnt[1].setFunction(new LinearFunction());
	st_rib9_L_T9_r9L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib9_L_T9_r9L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T9_r9L_Z", 1, 1));
	st_rib9_L_T9_r9L_CVjnt[2].setFunction(new LinearFunction());
	st_rib9_L_T9_r9L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T9_r9L_CVjnt = new CustomJoint("T9_r9L_CVjnt", *thoracic9, Vec3(-0.011492, 0.024682, -0.014417), Vec3(1.5708, 3.1416, -0.45172), *rib9_L, Vec3(0, 0, 0), Vec3(-1.5708, -0.12217, 2.6899), st_rib9_L_T9_r9L_CVjnt);
	model->addJoint(T9_r9L_CVjnt);
	//
	//
	Coordinate& T9_r9L_X = T9_r9L_CVjnt->upd_coordinates(0);
	T9_r9L_X.setDefaultLocked(true);
	T9_r9L_X.setDefaultClamped(false);
	//
	Coordinate& T9_r9L_Y = T9_r9L_CVjnt->upd_coordinates(1);
	T9_r9L_Y.setDefaultLocked(true);
	T9_r9L_Y.setDefaultClamped(false);
	//
	Coordinate& T9_r9L_Z = T9_r9L_CVjnt->upd_coordinates(2);
	T9_r9L_Z.setDefaultLocked(true);
	T9_r9L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib8_L_T8_r8L_CVjnt;
	st_rib8_L_T8_r8L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T8_r8L_X", 1, 1));
	st_rib8_L_T8_r8L_CVjnt[0].setFunction(new LinearFunction());
	st_rib8_L_T8_r8L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib8_L_T8_r8L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T8_r8L_Y", 1, 1));
	st_rib8_L_T8_r8L_CVjnt[1].setFunction(new LinearFunction());
	st_rib8_L_T8_r8L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib8_L_T8_r8L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T8_r8L_Z", 1, 1));
	st_rib8_L_T8_r8L_CVjnt[2].setFunction(new LinearFunction());
	st_rib8_L_T8_r8L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T8_r8L_CVjnt = new CustomJoint("T8_r8L_CVjnt", *thoracic8, Vec3(-0.010307, 0.024171, -0.014319), Vec3(1.5708, 3.1416, -0.49691), *rib8_L, Vec3(0, 0, 0), Vec3(-1.5708, -0.034907, 2.6447), st_rib8_L_T8_r8L_CVjnt);
	model->addJoint(T8_r8L_CVjnt);
	//
	//
	Coordinate& T8_r8L_X = T8_r8L_CVjnt->upd_coordinates(0);
	T8_r8L_X.setDefaultLocked(true);
	T8_r8L_X.setDefaultClamped(false);
	//
	Coordinate& T8_r8L_Y = T8_r8L_CVjnt->upd_coordinates(1);
	T8_r8L_Y.setDefaultLocked(true);
	T8_r8L_Y.setDefaultClamped(false);
	//
	Coordinate& T8_r8L_Z = T8_r8L_CVjnt->upd_coordinates(2);
	T8_r8L_Z.setDefaultLocked(true);
	T8_r8L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib7_L_T7_r7L_CVjnt;
	st_rib7_L_T7_r7L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T7_r7L_X", 1, 1));
	st_rib7_L_T7_r7L_CVjnt[0].setFunction(new LinearFunction());
	st_rib7_L_T7_r7L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib7_L_T7_r7L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T7_r7L_Y", 1, 1));
	st_rib7_L_T7_r7L_CVjnt[1].setFunction(new LinearFunction());
	st_rib7_L_T7_r7L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib7_L_T7_r7L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T7_r7L_Z", 1, 1));
	st_rib7_L_T7_r7L_CVjnt[2].setFunction(new LinearFunction());
	st_rib7_L_T7_r7L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T7_r7L_CVjnt = new CustomJoint("T7_r7L_CVjnt", *thoracic7, Vec3(-0.0098876, 0.025467, -0.01539), Vec3(1.5708, 3.1416, -0.53682), *rib7_L, Vec3(0, 0, 0), Vec3(-1.5708, 0.069813, 2.6048), st_rib7_L_T7_r7L_CVjnt);
	model->addJoint(T7_r7L_CVjnt);
	//
	//
	Coordinate& T7_r7L_X = T7_r7L_CVjnt->upd_coordinates(0);
	T7_r7L_X.setDefaultLocked(true);
	T7_r7L_X.setDefaultClamped(false);
	//
	Coordinate& T7_r7L_Y = T7_r7L_CVjnt->upd_coordinates(1);
	T7_r7L_Y.setDefaultLocked(true);
	T7_r7L_Y.setDefaultClamped(false);
	//
	Coordinate& T7_r7L_Z = T7_r7L_CVjnt->upd_coordinates(2);
	T7_r7L_Z.setDefaultLocked(true);
	T7_r7L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib6_L_T6_r6L_CVjnt;
	st_rib6_L_T6_r6L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T6_r6L_X", 1, 1));
	st_rib6_L_T6_r6L_CVjnt[0].setFunction(new LinearFunction());
	st_rib6_L_T6_r6L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib6_L_T6_r6L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T6_r6L_Y", 1, 1));
	st_rib6_L_T6_r6L_CVjnt[1].setFunction(new LinearFunction());
	st_rib6_L_T6_r6L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib6_L_T6_r6L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T6_r6L_Z", 1, 1));
	st_rib6_L_T6_r6L_CVjnt[2].setFunction(new LinearFunction());
	st_rib6_L_T6_r6L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T6_r6L_CVjnt = new CustomJoint("T6_r6L_CVjnt", *thoracic6, Vec3(-0.0083438, 0.024198, -0.014764), Vec3(1.5708, 3.1416, -0.57221), *rib6_L, Vec3(0, 0, 0), Vec3(-1.5708, 0.17453, 2.5694), st_rib6_L_T6_r6L_CVjnt);
	model->addJoint(T6_r6L_CVjnt);
	//
	//
	Coordinate& T6_r6L_X = T6_r6L_CVjnt->upd_coordinates(0);
	T6_r6L_X.setDefaultLocked(true);
	T6_r6L_X.setDefaultClamped(false);
	//
	Coordinate& T6_r6L_Y = T6_r6L_CVjnt->upd_coordinates(1);
	T6_r6L_Y.setDefaultLocked(true);
	T6_r6L_Y.setDefaultClamped(false);
	//
	Coordinate& T6_r6L_Z = T6_r6L_CVjnt->upd_coordinates(2);
	T6_r6L_Z.setDefaultLocked(true);
	T6_r6L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib5_L_T5_r5L_CVjnt;
	st_rib5_L_T5_r5L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T5_r5L_X", 1, 1));
	st_rib5_L_T5_r5L_CVjnt[0].setFunction(new LinearFunction());
	st_rib5_L_T5_r5L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib5_L_T5_r5L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T5_r5L_Y", 1, 1));
	st_rib5_L_T5_r5L_CVjnt[1].setFunction(new LinearFunction());
	st_rib5_L_T5_r5L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib5_L_T5_r5L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T5_r5L_Z", 1, 1));
	st_rib5_L_T5_r5L_CVjnt[2].setFunction(new LinearFunction());
	st_rib5_L_T5_r5L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T5_r5L_CVjnt = new CustomJoint("T5_r5L_CVjnt", *thoracic5, Vec3(-0.0074323, 0.024759, -0.015236), Vec3(1.5708, 3.1416, -0.60373), *rib5_L, Vec3(0, 0, 0), Vec3(-1.5708, 0.27925, 2.5379), st_rib5_L_T5_r5L_CVjnt);
	model->addJoint(T5_r5L_CVjnt);
	//
	//
	Coordinate& T5_r5L_X = T5_r5L_CVjnt->upd_coordinates(0);
	T5_r5L_X.setDefaultLocked(true);
	T5_r5L_X.setDefaultClamped(false);
	//
	Coordinate& T5_r5L_Y = T5_r5L_CVjnt->upd_coordinates(1);
	T5_r5L_Y.setDefaultLocked(true);
	T5_r5L_Y.setDefaultClamped(false);
	//
	Coordinate& T5_r5L_Z = T5_r5L_CVjnt->upd_coordinates(2);
	T5_r5L_Z.setDefaultLocked(true);
	T5_r5L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib4_L_T4_r4L_CVjnt;
	st_rib4_L_T4_r4L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T4_r4L_X", 1, 1));
	st_rib4_L_T4_r4L_CVjnt[0].setFunction(new LinearFunction());
	st_rib4_L_T4_r4L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib4_L_T4_r4L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T4_r4L_Y", 1, 1));
	st_rib4_L_T4_r4L_CVjnt[1].setFunction(new LinearFunction());
	st_rib4_L_T4_r4L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib4_L_T4_r4L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T4_r4L_Z", 1, 1));
	st_rib4_L_T4_r4L_CVjnt[2].setFunction(new LinearFunction());
	st_rib4_L_T4_r4L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T4_r4L_CVjnt = new CustomJoint("T4_r4L_CVjnt", *thoracic4, Vec3(-0.0057726, 0.02307, -0.014063), Vec3(1.5708, 3.1416, -0.63193), *rib4_L, Vec3(0, 0, 0), Vec3(-1.5708, 0.38397, 2.5097), st_rib4_L_T4_r4L_CVjnt);
	model->addJoint(T4_r4L_CVjnt);
	//
	//
	Coordinate& T4_r4L_X = T4_r4L_CVjnt->upd_coordinates(0);
	T4_r4L_X.setDefaultLocked(true);
	T4_r4L_X.setDefaultClamped(false);
	//
	Coordinate& T4_r4L_Y = T4_r4L_CVjnt->upd_coordinates(1);
	T4_r4L_Y.setDefaultLocked(true);
	T4_r4L_Y.setDefaultClamped(false);
	//
	Coordinate& T4_r4L_Z = T4_r4L_CVjnt->upd_coordinates(2);
	T4_r4L_Z.setDefaultLocked(true);
	T4_r4L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib3_L_T3_r3L_CVjnt;
	st_rib3_L_T3_r3L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T3_r3L_X", 1, 1));
	st_rib3_L_T3_r3L_CVjnt[0].setFunction(new LinearFunction());
	st_rib3_L_T3_r3L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib3_L_T3_r3L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T3_r3L_Y", 1, 1));
	st_rib3_L_T3_r3L_CVjnt[1].setFunction(new LinearFunction());
	st_rib3_L_T3_r3L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib3_L_T3_r3L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T3_r3L_Z", 1, 1));
	st_rib3_L_T3_r3L_CVjnt[2].setFunction(new LinearFunction());
	st_rib3_L_T3_r3L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T3_r3L_CVjnt = new CustomJoint("T3_r3L_CVjnt", *thoracic3, Vec3(-0.004191, 0.020935, -0.012579), Vec3(1.5708, 3.1416, -0.65727), *rib3_L, Vec3(0, 0, 0), Vec3(-1.5708, 0.45379, 2.4843), st_rib3_L_T3_r3L_CVjnt);
	model->addJoint(T3_r3L_CVjnt);
	//
	//
	Coordinate& T3_r3L_X = T3_r3L_CVjnt->upd_coordinates(0);
	T3_r3L_X.setDefaultLocked(true);
	T3_r3L_X.setDefaultClamped(false);
	//
	Coordinate& T3_r3L_Y = T3_r3L_CVjnt->upd_coordinates(1);
	T3_r3L_Y.setDefaultLocked(true);
	T3_r3L_Y.setDefaultClamped(false);
	//
	Coordinate& T3_r3L_Z = T3_r3L_CVjnt->upd_coordinates(2);
	T3_r3L_Z.setDefaultLocked(true);
	T3_r3L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib2_L_T2_r2L_CVjnt;
	st_rib2_L_T2_r2L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T2_r2L_X", 1, 1));
	st_rib2_L_T2_r2L_CVjnt[0].setFunction(new LinearFunction());
	st_rib2_L_T2_r2L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib2_L_T2_r2L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T2_r2L_Y", 1, 1));
	st_rib2_L_T2_r2L_CVjnt[1].setFunction(new LinearFunction());
	st_rib2_L_T2_r2L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib2_L_T2_r2L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T2_r2L_Z", 1, 1));
	st_rib2_L_T2_r2L_CVjnt[2].setFunction(new LinearFunction());
	st_rib2_L_T2_r2L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T2_r2L_CVjnt = new CustomJoint("T2_r2L_CVjnt", *thoracic2, Vec3(-0.0028935, 0.019107, -0.011309), Vec3(1.5708, 3.1416, -0.68013), *rib2_L, Vec3(0, 0, 0), Vec3(-1.5708, 0.50615, 2.4615), st_rib2_L_T2_r2L_CVjnt);
	model->addJoint(T2_r2L_CVjnt);
	//
	//
	Coordinate& T2_r2L_X = T2_r2L_CVjnt->upd_coordinates(0);
	T2_r2L_X.setDefaultLocked(true);
	T2_r2L_X.setDefaultClamped(false);
	//
	Coordinate& T2_r2L_Y = T2_r2L_CVjnt->upd_coordinates(1);
	T2_r2L_Y.setDefaultLocked(true);
	T2_r2L_Y.setDefaultClamped(false);
	//
	Coordinate& T2_r2L_Z = T2_r2L_CVjnt->upd_coordinates(2);
	T2_r2L_Z.setDefaultLocked(true);
	T2_r2L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_rib1_L_T1_r1L_CVjnt;
	st_rib1_L_T1_r1L_CVjnt[0].setCoordinateNames(OpenSim::Array<std::string>("T1_r1L_X", 1, 1));
	st_rib1_L_T1_r1L_CVjnt[0].setFunction(new LinearFunction());
	st_rib1_L_T1_r1L_CVjnt[0].setAxis(Vec3(1, 0, 0));
	st_rib1_L_T1_r1L_CVjnt[1].setCoordinateNames(OpenSim::Array<std::string>("T1_r1L_Y", 1, 1));
	st_rib1_L_T1_r1L_CVjnt[1].setFunction(new LinearFunction());
	st_rib1_L_T1_r1L_CVjnt[1].setAxis(Vec3(0, 1, 0));
	st_rib1_L_T1_r1L_CVjnt[2].setCoordinateNames(OpenSim::Array<std::string>("T1_r1L_Z", 1, 1));
	st_rib1_L_T1_r1L_CVjnt[2].setFunction(new LinearFunction());
	st_rib1_L_T1_r1L_CVjnt[2].setAxis(Vec3(0, 0, 1));
	T1_r1L_CVjnt = new CustomJoint("T1_r1L_CVjnt", *thoracic1, Vec3(-0.0031561, 0.018379, -0.017678), Vec3(1.5708, 3.1416, -0.70085), *rib1_L, Vec3(0, 0, 0), Vec3(-1.5708, 0.5236, 2.4407), st_rib1_L_T1_r1L_CVjnt);
	model->addJoint(T1_r1L_CVjnt);
	//
	//
	Coordinate& T1_r1L_X = T1_r1L_CVjnt->upd_coordinates(0);
	T1_r1L_X.setDefaultLocked(true);
	T1_r1L_X.setDefaultClamped(false);
	//
	Coordinate& T1_r1L_Y = T1_r1L_CVjnt->upd_coordinates(1);
	T1_r1L_Y.setDefaultLocked(true);
	T1_r1L_Y.setDefaultClamped(false);
	//
	Coordinate& T1_r1L_Z = T1_r1L_CVjnt->upd_coordinates(2);
	T1_r1L_Z.setDefaultLocked(true);
	T1_r1L_Z.setDefaultClamped(false);
	model->updJointSet();
	//
	SpatialTransform st_sternum_r1R_sterR_jnt;
	st_sternum_r1R_sterR_jnt[0].setCoordinateNames(OpenSim::Array<std::string>("SternumRotZ", 1, 1));
	st_sternum_r1R_sterR_jnt[0].setFunction(new LinearFunction());
	st_sternum_r1R_sterR_jnt[0].setAxis(Vec3(0, 0, 1));
	st_sternum_r1R_sterR_jnt[1].setCoordinateNames(OpenSim::Array<std::string>("SternumRotX", 1, 1));
	st_sternum_r1R_sterR_jnt[1].setFunction(new LinearFunction());
	st_sternum_r1R_sterR_jnt[1].setAxis(Vec3(1, 0, 0));
	st_sternum_r1R_sterR_jnt[2].setCoordinateNames(OpenSim::Array<std::string>("SternumRotY", 1, 1));
	st_sternum_r1R_sterR_jnt[2].setFunction(new LinearFunction());
	st_sternum_r1R_sterR_jnt[2].setAxis(Vec3(0, 1, 0));
	st_sternum_r1R_sterR_jnt[3].setCoordinateNames(OpenSim::Array<std::string>("SternumX", 1, 1));
	st_sternum_r1R_sterR_jnt[3].setFunction(new LinearFunction());
	st_sternum_r1R_sterR_jnt[3].setAxis(Vec3(1, 0, 0));
	st_sternum_r1R_sterR_jnt[4].setCoordinateNames(OpenSim::Array<std::string>("SternumY", 1, 1));
	st_sternum_r1R_sterR_jnt[4].setFunction(new LinearFunction());
	st_sternum_r1R_sterR_jnt[4].setAxis(Vec3(0, 1, 0));
	st_sternum_r1R_sterR_jnt[5].setCoordinateNames(OpenSim::Array<std::string>("SternumZ", 1, 1));
	st_sternum_r1R_sterR_jnt[5].setFunction(new LinearFunction());
	st_sternum_r1R_sterR_jnt[5].setAxis(Vec3(0, 0, 1));
	r1R_sterR_jnt = new CustomJoint("r1R_sterR_jnt", *rib1_R, Vec3(0.064511, -0.05161, 0.013945), Vec3(0, 0, 0), *sternum, Vec3(0, 0, 0), Vec3(0, 0, 0), st_sternum_r1R_sterR_jnt);
	model->addJoint(r1R_sterR_jnt);
	//
	//
	Coordinate& SternumRotZ = r1R_sterR_jnt->upd_coordinates(0);
	SternumRotZ.setDefaultLocked(true);
	SternumRotZ.setDefaultClamped(false);
	//
	Coordinate& SternumRotX = r1R_sterR_jnt->upd_coordinates(1);
	SternumRotX.setDefaultLocked(true);
	SternumRotX.setDefaultClamped(false);
	//
	Coordinate& SternumRotY = r1R_sterR_jnt->upd_coordinates(2);
	SternumRotY.setDefaultLocked(true);
	SternumRotY.setDefaultClamped(false);
	//
	Coordinate& SternumX = r1R_sterR_jnt->upd_coordinates(3);
	SternumX.setDefaultLocked(true);
	SternumX.setDefaultClamped(false);
	//
	Coordinate& SternumY = r1R_sterR_jnt->upd_coordinates(4);
	SternumY.setDefaultLocked(true);
	SternumY.setDefaultClamped(false);
	//
	Coordinate& SternumZ = r1R_sterR_jnt->upd_coordinates(5);
	SternumZ.setDefaultLocked(true);
	SternumZ.setDefaultClamped(false);
	model->updJointSet();
	//
	sterR_clavR_jnt_weld = new WeldJoint("sterR_clavR_jnt_weld", *sternum, Vec3(0, 0, 0), Vec3(0, -0.2618, 0), *clavicle_R, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(sterR_clavR_jnt_weld);
	//
	clavR_scapR_jnt_weld = new WeldJoint("clavR_scapR_jnt_weld", *clavicle_R, Vec3(-0.013284, 0.018606, 0.12565), Vec3(0, 0.34907, 0), *scapula_R, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(clavR_scapR_jnt_weld);
	//
	SpatialTransform st_humerus_R_shoulder_R;
	st_humerus_R_shoulder_R[0].setCoordinateNames(OpenSim::Array<std::string>("shoulder_elv_r", 1, 1));
	st_humerus_R_shoulder_R[0].setFunction(new LinearFunction());
	st_humerus_R_shoulder_R[0].setAxis(Vec3(-0.99826, 0.0023, 0.058898));
	st_humerus_R_shoulder_R[1].setCoordinateNames(OpenSim::Array<std::string>("shoulder_rot_r", 1, 1));
	st_humerus_R_shoulder_R[1].setFunction(new LinearFunction());
	st_humerus_R_shoulder_R[1].setAxis(Vec3(0.0048, 0.99909, 0.0424));
	st_humerus_R_shoulder_R[2].setCoordinateNames(OpenSim::Array<std::string>("elv_angle_r", 1, 1));
	st_humerus_R_shoulder_R[2].setFunction(new LinearFunction());
	st_humerus_R_shoulder_R[2].setAxis(Vec3(0.0048, 0.0424, 0.99909));
	shoulder_R = new CustomJoint("shoulder_R", *scapula_R, Vec3(-0.0084788, -0.030045, 0.007953), Vec3(0, -0.087266, 0), *humerus_R, Vec3(0, 0, 0), Vec3(0, 0, 0), st_humerus_R_shoulder_R);
	model->addJoint(shoulder_R);
	//
	//
	Coordinate& shoulder_elv_r = shoulder_R->upd_coordinates(0);
	shoulder_elv_r.setDefaultLocked(false);
	shoulder_elv_r.setDefaultClamped(true);
	//
	Coordinate& shoulder_rot_r = shoulder_R->upd_coordinates(1);
	shoulder_rot_r.setDefaultLocked(false);
	shoulder_rot_r.setDefaultClamped(true);
	//
	Coordinate& elv_angle_r = shoulder_R->upd_coordinates(2);
	elv_angle_r.setDefaultLocked(false);
	elv_angle_r.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_ulna_R_elbow;
	st_ulna_R_elbow[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flexion_r", 1, 1));
	st_ulna_R_elbow[0].setFunction(new LinearFunction());
	st_ulna_R_elbow[0].setAxis(Vec3(0.0494, 0.0366, 0.99811));
	elbow = new CustomJoint("elbow", *humerus_R, Vec3(0.0052496, -0.24991, -0.010585), Vec3(0, 0, 0), *ulna_R, Vec3(0, 0, 0), Vec3(0, 0, 0), st_ulna_R_elbow);
	model->addJoint(elbow);
	//
	//
	Coordinate& elbow_flexion_r = elbow->upd_coordinates(0);
	elbow_flexion_r.setDefaultLocked(false);
	elbow_flexion_r.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_radius_R_radioulnar;
	st_radius_R_radioulnar[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_r", 1, 1));
	st_radius_R_radioulnar[0].setFunction(new LinearFunction());
	st_radius_R_radioulnar[0].setAxis(Vec3(-0.017161, 0.99267, -0.11967));
	radioulnar = new CustomJoint("radioulnar", *ulna_R, Vec3(0.00034423, -0.0098993, 0.017211), Vec3(0, 0, 0), *radius_R, Vec3(0, 0, 0), Vec3(0, 0, 0), st_radius_R_radioulnar);
	model->addJoint(radioulnar);
	//
	//
	Coordinate& pro_sup_r = radioulnar->upd_coordinates(0);
	pro_sup_r.setDefaultLocked(true);
	pro_sup_r.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_hand_R_radius_hand_r;
	st_hand_R_radius_hand_r[0].setCoordinateNames(OpenSim::Array<std::string>("wrist_dev_r", 1, 1));
	st_hand_R_radius_hand_r[0].setFunction(new LinearFunction());
	st_hand_R_radius_hand_r[0].setAxis(Vec3(-0.81906, -0.13561, -0.55744));
	st_hand_R_radius_hand_r[1].setCoordinateNames(OpenSim::Array<std::string>("wrist_flex_r", 1, 1));
	st_hand_R_radius_hand_r[1].setFunction(new LinearFunction());
	st_hand_R_radius_hand_r[1].setAxis(Vec3(0.95643, -0.25221, 0.1471));
	radius_hand_r = new CustomJoint("radius_hand_r", *radius_R, Vec3(0.015491, -0.20826, 0.021515), Vec3(0, 0, 0), *hand_R, Vec3(0, 0, 0), Vec3(0, 0, 0), st_hand_R_radius_hand_r);
	model->addJoint(radius_hand_r);
	//
	//
	Coordinate& wrist_dev_r = radius_hand_r->upd_coordinates(0);
	wrist_dev_r.setDefaultLocked(true);
	wrist_dev_r.setDefaultClamped(true);
	//
	Coordinate& wrist_flex_r = radius_hand_r->upd_coordinates(1);
	wrist_flex_r.setDefaultLocked(true);
	wrist_flex_r.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_humerus_L_shoulder_L;
	st_humerus_L_shoulder_L[0].setCoordinateNames(OpenSim::Array<std::string>("shoulder_elv_l", 1, 1));
	st_humerus_L_shoulder_L[0].setFunction(new LinearFunction());
	st_humerus_L_shoulder_L[0].setAxis(Vec3(-0.99826, 0.0023, 0.058898));
	st_humerus_L_shoulder_L[1].setCoordinateNames(OpenSim::Array<std::string>("shoulder_rot_l", 1, 1));
	st_humerus_L_shoulder_L[1].setFunction(new LinearFunction());
	st_humerus_L_shoulder_L[1].setAxis(Vec3(0.0048, 0.99909, 0.0424));
	st_humerus_L_shoulder_L[2].setCoordinateNames(OpenSim::Array<std::string>("elv_angle_l", 1, 1));
	st_humerus_L_shoulder_L[2].setFunction(new LinearFunction());
	st_humerus_L_shoulder_L[2].setAxis(Vec3(0.0048, 0.0424, 0.99909));
	shoulder_L = new CustomJoint("shoulder_L", *scapula_L, Vec3(-0.0084788, -0.030045, -0.007953), Vec3(0, 0.087266, 0), *humerus_L, Vec3(0, 0, 0), Vec3(0, 0, 0), st_humerus_L_shoulder_L);
	model->addJoint(shoulder_L);
	//
	//
	Coordinate& shoulder_elv_l = shoulder_L->upd_coordinates(0);
	shoulder_elv_l.setDefaultLocked(false);
	shoulder_elv_l.setDefaultClamped(true);
	//
	Coordinate& shoulder_rot_l = shoulder_L->upd_coordinates(1);
	shoulder_rot_l.setDefaultLocked(false);
	shoulder_rot_l.setDefaultClamped(true);
	//
	Coordinate& elv_angle_l = shoulder_L->upd_coordinates(2);
	elv_angle_l.setDefaultLocked(false);
	elv_angle_l.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_ulna_L_elbow_l;
	st_ulna_L_elbow_l[0].setCoordinateNames(OpenSim::Array<std::string>("elbow_flexion_l", 1, 1));
	st_ulna_L_elbow_l[0].setFunction(new LinearFunction());
	st_ulna_L_elbow_l[0].setAxis(Vec3(-0.0494, -0.0366, 0.99811));
	elbow_l = new CustomJoint("elbow_l", *humerus_L, Vec3(0.0052496, -0.24991, 0.010585), Vec3(0, 0, 0), *ulna_L, Vec3(0, 0, 0), Vec3(0, 0, 0), st_ulna_L_elbow_l);
	model->addJoint(elbow_l);
	//
	//
	Coordinate& elbow_flexion_l = elbow_l->upd_coordinates(0);
	elbow_flexion_l.setDefaultLocked(false);
	elbow_flexion_l.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_radius_L_radioulnar_l;
	st_radius_L_radioulnar_l[0].setCoordinateNames(OpenSim::Array<std::string>("pro_sup_l", 1, 1));
	st_radius_L_radioulnar_l[0].setFunction(new LinearFunction());
	st_radius_L_radioulnar_l[0].setAxis(Vec3(0.017161, -0.99267, -0.11967));
	radioulnar_l = new CustomJoint("radioulnar_l", *ulna_L, Vec3(0.00034423, -0.0098993, -0.017211), Vec3(0, 0, 0), *radius_L, Vec3(0, 0, 0), Vec3(0, 0, 0), st_radius_L_radioulnar_l);
	model->addJoint(radioulnar_l);
	//
	//
	Coordinate& pro_sup_l = radioulnar_l->upd_coordinates(0);
	pro_sup_l.setDefaultLocked(true);
	pro_sup_l.setDefaultClamped(true);
	model->updJointSet();
	//
	SpatialTransform st_hand_L_radius_hand_l;
	st_hand_L_radius_hand_l[0].setCoordinateNames(OpenSim::Array<std::string>("wrist_dev_l", 1, 1));
	st_hand_L_radius_hand_l[0].setFunction(new LinearFunction());
	st_hand_L_radius_hand_l[0].setAxis(Vec3(-0.81906, -0.13561, -0.55744));
	st_hand_L_radius_hand_l[1].setCoordinateNames(OpenSim::Array<std::string>("wrist_flex_l", 1, 1));
	st_hand_L_radius_hand_l[1].setFunction(new LinearFunction());
	st_hand_L_radius_hand_l[1].setAxis(Vec3(0.95643, -0.25221, 0.1471));
	radius_hand_l = new CustomJoint("radius_hand_l", *radius_L, Vec3(0.015491, -0.20826, -0.021515), Vec3(0, 0, 0), *hand_L, Vec3(0, 0, 0), Vec3(0, 0, 0), st_hand_L_radius_hand_l);
	model->addJoint(radius_hand_l);
	//
	//
	Coordinate& wrist_dev_l = radius_hand_l->upd_coordinates(0);
	wrist_dev_l.setDefaultLocked(true);
	wrist_dev_l.setDefaultClamped(true);
	//
	Coordinate& wrist_flex_l = radius_hand_l->upd_coordinates(1);
	wrist_flex_l.setDefaultLocked(true);
	wrist_flex_l.setDefaultClamped(true);
	model->updJointSet();
	//
	sterL_clavL_jnt_weld = new WeldJoint("sterL_clavL_jnt_weld", *sternum, Vec3(0, 0, -0.061587), Vec3(0, 0.2618, 0), *clavicle_L, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(sterL_clavL_jnt_weld);
	//
	clavL_scapL_jnt_weld = new WeldJoint("clavL_scapL_jnt_weld", *clavicle_L, Vec3(-0.013284, 0.018606, -0.12565), Vec3(0, -0.34907, 0), *scapula_L, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(clavL_scapL_jnt_weld);
	//
	Abd_L_L1_weld = new WeldJoint("Abd_L_L1_weld", *lumbar1, Vec3(0, 0, 0), Vec3(0, 0, -0.38397), *Abd_L_L1, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_L_L1_weld);
	//
	Abd_L_L2_weld = new WeldJoint("Abd_L_L2_weld", *lumbar2, Vec3(0, 0, 0), Vec3(0, 0, -0.27925), *Abd_L_L2, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_L_L2_weld);
	//
	Abd_L_L3_weld = new WeldJoint("Abd_L_L3_weld", *lumbar3, Vec3(0, 0, 0), Vec3(0, 0, -0.15708), *Abd_L_L3, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_L_L3_weld);
	//
	Abd_L_L4_weld = new WeldJoint("Abd_L_L4_weld", *lumbar4, Vec3(0, 0, 0), Vec3(0, 0, 0.05236), *Abd_L_L4, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_L_L4_weld);
	//
	Abd_L_L5_weld = new WeldJoint("Abd_L_L5_weld", *lumbar5, Vec3(0, 0, 0), Vec3(0, 0, 0.36652), *Abd_L_L5, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_L_L5_weld);
	//
	Abd_R_L1_weld = new WeldJoint("Abd_R_L1_weld", *lumbar1, Vec3(0, 0, 0), Vec3(0, 0, -0.38397), *Abd_R_L1, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_R_L1_weld);
	//
	Abd_R_L2_weld = new WeldJoint("Abd_R_L2_weld", *lumbar2, Vec3(0, 0, 0), Vec3(0, 0, -0.27925), *Abd_R_L2, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_R_L2_weld);
	//
	Abd_R_L3_weld = new WeldJoint("Abd_R_L3_weld", *lumbar3, Vec3(0, 0, 0), Vec3(0, 0, -0.15708), *Abd_R_L3, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_R_L3_weld);
	//
	Abd_R_L4_weld = new WeldJoint("Abd_R_L4_weld", *lumbar4, Vec3(0, 0, 0), Vec3(0, 0, 0.05236), *Abd_R_L4, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_R_L4_weld);
	//
	Abd_R_L5_weld = new WeldJoint("Abd_R_L5_weld", *lumbar5, Vec3(0, 0, 0), Vec3(0, 0, 0.36652), *Abd_R_L5, Vec3(0, 0, 0), Vec3(0, 0, 0));
	model->addJoint(Abd_R_L5_weld);
	//
	//C7 = new OpenSim::Marker("C7", *cerv7, Vec3(-0.03317, 0.029014, -0.0033559));
	//model->addMarker(C7);
	//T1L = new OpenSim::Marker("T1L", *thoracic1, Vec3(-0.058393, 0.0095075, -0.019223));
	//model->addMarker(T1L);
	//T1M = new OpenSim::Marker("T1M", *thoracic1, Vec3(-0.083942, 0.0070414, -0.0022272));
	//model->addMarker(T1M);
	//T1R = new OpenSim::Marker("T1R", *thoracic1, Vec3(-0.059191, 0.025877, 0.014934));
	//model->addMarker(T1R);
	//T3M = new OpenSim::Marker("T3M", *thoracic3, Vec3(-0.091803, 0.017763, 0.0032816));
	//model->addMarker(T3M);
	//T3L = new OpenSim::Marker("T3L", *thoracic3, Vec3(-0.067906, 0.016478, -0.014612));
	//model->addMarker(T3L);
	//T3R = new OpenSim::Marker("T3R", *thoracic3, Vec3(-0.063142, 0.029635, 0.021415));
	//model->addMarker(T3R);
	//T5 = new OpenSim::Marker("T5", *thoracic5, Vec3(-0.059088, -0.007026, 0.00014894));
	//model->addMarker(T5);
	//T7M = new OpenSim::Marker("T7M", *thoracic7, Vec3(-0.093138, 0.0061751, -0.0047169));
	//model->addMarker(T7M);
	//T7L = new OpenSim::Marker("T7L", *thoracic7, Vec3(-0.066914, 0.0070341, -0.019495));
	//model->addMarker(T7L);
	//T7R = new OpenSim::Marker("T7R", *thoracic7, Vec3(-0.06932, 0.022773, 0.016349));
	//model->addMarker(T7R);
	//T9 = new OpenSim::Marker("T9", *thoracic9, Vec3(-0.069146, 0.022933, -0.0008087));
	//model->addMarker(T9);
	//T11M = new OpenSim::Marker("T11M", *thoracic11, Vec3(-0.11215, 0.011037, 0.0031922));
	//model->addMarker(T11M);
	//T11L = new OpenSim::Marker("T11L", *thoracic11, Vec3(-0.090603, -0.0094492, -0.016764));
	//model->addMarker(T11L);
	//T11R = new OpenSim::Marker("T11R", *thoracic11, Vec3(-0.085857, 0.0079551, 0.017529));
	//model->addMarker(T11R);
	//T12 = new OpenSim::Marker("T12", *thoracic12, Vec3(-0.082568, 0.01901, -0.0015304));
	//model->addMarker(T12);
	//L2M = new OpenSim::Marker("L2M", *lumbar2, Vec3(-0.085328, 0.011443, 0.015647));
	//model->addMarker(L2M);
	//L2L = new OpenSim::Marker("L2L", *lumbar2, Vec3(-0.10498, -0.015693, 0.0032577));
	//model->addMarker(L2L);
	//L2R = new OpenSim::Marker("L2R", *lumbar2, Vec3(-0.082745, -0.010804, -0.016435));
	//model->addMarker(L2R);
	//L3 = new OpenSim::Marker("L3", *lumbar3, Vec3(-0.073728, 0.022114, 0.0010705));
	//model->addMarker(L3);
	//L4M = new OpenSim::Marker("L4M", *lumbar4, Vec3(-0.080276, 0.0088236, 0.021169));
	//model->addMarker(L4M);
	//L4L = new OpenSim::Marker("L4L", *lumbar4, Vec3(-0.10136, -0.015625, 0.0057773));
	//model->addMarker(L4L);
	//SACR = new OpenSim::Marker("SACR", *sacrum, Vec3(-0.15085, 0.063101, 0.0083329));
	//model->addMarker(SACR);
	//L4R = new OpenSim::Marker("L4R", *lumbar4, Vec3(-0.079989, -0.008168, -0.013757));
	//model->addMarker(L4R);
	//RPSI = new OpenSim::Marker("RPSI", *sacrum, Vec3(-0.14524, 0.053107, 0.051865));
	//model->addMarker(RPSI);
	//LPSI = new OpenSim::Marker("LPSI", *sacrum, Vec3(-0.15475, 0.072111, -0.037615));
	//model->addMarker(LPSI);
	//LASI = new OpenSim::Marker("LASI", *sacrum, Vec3(-0.01138, 0.062783, -0.12996));
	//model->addMarker(LASI);
	//RASI = new OpenSim::Marker("RASI", *pelvis, Vec3(0.010924, -0.016486, 0.1195));
	//model->addMarker(RASI);
	//STRN = new OpenSim::Marker("STRN", *sternum, Vec3(0.045392, -0.15841, -0.027781));
	//model->addMarker(STRN);
	//LFHD = new OpenSim::Marker("LFHD", *skull, Vec3(0.073097, 0.075717, -0.048222));
	//model->addMarker(LFHD);
	//RFHD = new OpenSim::Marker("RFHD", *skull, Vec3(0.073634, 0.074551, 0.053249));
	//model->addMarker(RFHD);
	//LBHD = new OpenSim::Marker("LBHD", *skull, Vec3(-0.083387, 0.035035, -0.054373));
	//model->addMarker(LBHD);
	//RBHD = new OpenSim::Marker("RBHD", *skull, Vec3(-0.081677, 0.03641, 0.052032));
	//model->addMarker(RBHD);
	//LSHO = new OpenSim::Marker("LSHO", *scapula_L, Vec3(-0.014358, 0.005532, -0.026024));
	//model->addMarker(LSHO);
	//RSHO = new OpenSim::Marker("RSHO", *scapula_R, Vec3(0.01106, 0.005302, 0.024372));
	//model->addMarker(RSHO);
	//CLAV = new OpenSim::Marker("CLAV", *sternum, Vec3(0.0046492, -0.0076517, -0.033781));
	//model->addMarker(CLAV);
	//RBAK = new OpenSim::Marker("RBAK", *scapula_R, Vec3(-0.088262, -0.040048, -0.059082));
	//model->addMarker(RBAK);
	//T3AA = new OpenSim::Marker("T3AA", *thoracic3, Vec3(-0.040142, 0.024487, -0.006086));
	//model->addMarker(T3AA);
	//T7AA = new OpenSim::Marker("T7AA", *thoracic7, Vec3(-0.043032, 0.025624, 0.0044115));
	//model->addMarker(T7AA);
	//T11AA = new OpenSim::Marker("T11AA", *thoracic11, Vec3(-0.05769, -0.013835, -0.0011));
	//model->addMarker(T11AA);
	//L2AA = new OpenSim::Marker("L2AA", *lumbar2, Vec3(-0.08657, -0.046818, 0.00027232));
	//model->addMarker(L2AA);
	//L4AA = new OpenSim::Marker("L4AA", *lumbar4, Vec3(-0.084046, -0.046319, -0.0028599));
	//model->addMarker(L4AA);
	//LELB = new OpenSim::Marker("LELB", *humerus_L, Vec3(0.017775, -0.25231, 0.0042893));
	//model->addMarker(LELB);
	//LELM = new OpenSim::Marker("LELM", *humerus_L, Vec3(0.0080328, -0.25437, 0.070859));
	//model->addMarker(LELM);
	//RELB = new OpenSim::Marker("RELB", *humerus_R, Vec3(0.025269, -0.26248, -0.011237));
	//model->addMarker(RELB);
	//RELM = new OpenSim::Marker("RELM", *humerus_R, Vec3(0.0086284, -0.26331, -0.069535));
	//model->addMarker(RELM);
	//LWRA = new OpenSim::Marker("LWRA", *radius_L, Vec3(0.049413, -0.23388, 0.01209));
	//model->addMarker(LWRA);
	//LWRB = new OpenSim::Marker("LWRB", *ulna_L, Vec3(-0.041556, -0.24153, -0.0081361));
	//model->addMarker(LWRB);
	//RWRA = new OpenSim::Marker("RWRA", *radius_R, Vec3(0.038585, -0.24134, -0.016777));
	//model->addMarker(RWRA);
	//RWRB = new OpenSim::Marker("RWRB", *ulna_R, Vec3(-0.047076, -0.24793, 0.034949));
	//model->addMarker(RWRB);

	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////

	state = new State(model->initSystem());
	//markerSet = new OpenSim::MarkerSet(model->getMarkerSet());

	// Get state order in Simbody and OpenSim
	auto s = model->getWorkingState();
	const auto svNames = model->getStateVariableNames();
	s.updQ() = 0;
	idx_states = new int[s.getNQ()];
	for (int iy = 0; iy < s.getNQ(); ++iy) {
		s.updQ()[iy] = SimTK::NaN;
		const auto svValues = model->getStateVariableValues(s);
		for (int isv = 0; isv < svNames.size(); ++isv) {
			if (SimTK::isNaN(svValues[isv])) {
				s.updQ()[iy] = 0;
				idx_states[iy] = isv / 2;
				break;
			}
		}
	}
	idx_states2 = new int[s.getNQ()];
	for (int iy = 0; iy < s.getNQ(); ++iy) {
		for (int iyy = 0; iyy < s.getNQ(); ++iyy) {
			if (idx_states[iyy] == iy) {
				idx_states2[iy] = iyy;
				break;
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////


	// Read inputs
	std::vector<T> x(arg[0], arg[0] + NQall * 2);
	//	std::vector<T> x(arg[0], arg[0] + NX);
	std::vector<T> u(arg[1], arg[1] + NQall);

	// marker indexs
	std::vector<T> EBBEw(arg[2], arg[2] + nEBBEw);
	T EBBEwMat[nEBBEw] = {};
	for (int i = 0; i < nEBBEw; ++i) EBBEwMat[i] = EBBEw[i];


	// States and controls
	static T ua[NQall]; /// joint accelerations (Qdotdots) - controls
	Vector QsUs(NQall * 2); /// joint positions (Qs) and velocities (Us) - states
	for (int i = 0; i < NX; ++i) QsUs[i] = x[i];    // tracked joints (Q and Qdot) index in ALL joints, in Opensim


	T uall[NQall] = {};
	for (int i = 0; i < NQall; ++i) uall[i] = u[i];
	for (int i = 0; i < NQall; ++i) ua[i] = uall[idx_states[i]];  //order in OpenSim is transferred to Simbody

	model->setStateVariableValues(*state, QsUs); //using OpenSim function instead of using SetQ, which is Simbody Function
	model->realizeVelocity(*state);

	////Marker position
	////static String MkrNamesOSIM[25] = { "T1","T3M","T3L","T3R","T5","T7M","T7L","T7R","T9","T11M","T11L","T11R", "T12",
	////"L2M","L2L","L2R","L3","L4M","L4L","L4R", "STRN","CLAV","LSHO","RSHO","RBAK" };
	//double MkrGlobalDataMat[nMkr][3];
	////marker position
	//Vec3 MkrGlobalPos;
	//for (int i = 0; i < nMkr; ++i) {
	//	MkrGlobalPos = model->getMarkerSet().get(MkrNamesOSIM[i]).getLocationInGround(*state);
	//	for (int k = 0; k < 3; ++k) {
	//		MkrGlobalDataMat[i][k] = MkrGlobalPos.get(k);
	//	}
	//}
	//
	//// Residual forces
	///// appliedMobilityForces (# mobilities)
	//Vector appliedMobilityForces(ndofr);
	//appliedMobilityForces.setToZero();
	///// appliedBodyForces (# bodies + ground)
	//Vector_<SpatialVec> appliedBodyForces;
	//int nbodies = model->getBodySet().getSize() + 1; // including ground
	//appliedBodyForces.resize(nbodies);

	//appliedBodyForces.setToZero();
	///// Gravity
	//Vec3 gravity(0);
	//gravity[1] = -9.81;
	///// Weight
	//for (int i = 0; i < model->getBodySet().getSize(); ++i) {
	//	model->getMatterSubsystem().addInStationForce(*state,
	//		model->getBodySet().get(i).getMobilizedBodyIndex(),
	//		model->getBodySet().get(i).getMassCenter(),
	//		model->getBodySet().get(i).getMass()*gravity, appliedBodyForces);
	//}
	//////////////////////////////////////////////////////////////////////////////////
	////Bushing forces
	////OpenSim::ForceSet& forceSet = model->updForceSet();

	////int numEBBE = 17;

	////for (int i = 0; i < numEBBE; ++i) {
	////	ExpressionBasedBushingForce* EBBEi = dynamic_cast<ExpressionBasedBushingForce*>(&forceSet.get(EBBElist[i]));


	////	// total bushing force in the internal basis of the deflection (dq)
	////	//Vec6 f = calcStiffnessForce(s) + calcDampingForce(s);//
	////	Vec6 f(EBBEwMat[1 + 6 * i], EBBEwMat[2 + 6 * i], EBBEwMat[0 + 6 * i], EBBEwMat[3 + 6 * i], EBBEwMat[4 + 6 * i], EBBEwMat[5 + 6 * i]);
	////	//   Mx/y/z, Fx/y/z

	////	SpatialVec F_GM(Vec3(0.0), Vec3(0.0));
	////	SpatialVec F_GF(Vec3(0.0), Vec3(0.0));

	////	//EBBEi->convertInternalForceToForcesOnFrames(*state, f, F_GF, F_GM);


	////	// internal force on body 2
	////	const SimTK::Vec3& fB2_q = f.getSubVec<3>(0); // in q basis
	////	const SimTK::Vec3& fM_F = f.getSubVec<3>(3); // acts at M, but exp. in F frame

	////	SimTK::Vec6 dq = EBBEi->computeDeflection(*state);   // maybe have to be two basic frame

	////														 //// get connected frames
	////														 //const F& frame1 = EBBEi->getFrame1();
	////														 //const F& frame2 = EBBEi->getFrame2();

	////														 //const SimTK::Transform& X_GB1 = frame1.getMobilizedBody().getBodyTransform(s);
	////														 //const SimTK::Transform& X_GB2 = frame2.getMobilizedBody().getBodyTransform(s);

	////	SimTK::Transform X_GF = EBBEi->getFrame1().getTransformInGround(*state); //.findBaseFrame().
	////	SimTK::Transform X_GM = EBBEi->getFrame2().getTransformInGround(*state);
	////	SimTK::Transform X_FM = ~X_GF * X_GM;
	////	const SimTK::Mat33 N_FM =
	////		SimTK::Rotation::calcNForBodyXYZInBodyFrame(dq.getSubVec<3>(0));

	////	// Calculate the matrix relating q-space generalized forces to a real-space
	////	// moment vector. We know qforce = ~H * moment (where H is the
	////	// the hinge matrix for a mobilizer using qdots as generalized speeds).
	////	// In that case H would be N^-1, qforce = ~(N^-1)*moment so
	////	// moment = ~N*qforce. Caution: our N wants the moment in the outboard
	////	// body frame, in this case M.
	////	const SimTK::Vec3  mB2_M = ~N_FM * fB2_q; // moment acting on body 2, exp. in M
	////	const SimTK::Vec3  mB2_G = X_GM.R() * mB2_M; // moment on body 2, now exp. in G

	////												 // Transform force from F frame to ground. This is the force to
	////												 // apply to body 2 at point OM; -f goes on body 1 at the same
	////												 // spatial location. Here we actually apply it at OF so we have to
	////												 // account for the moment produced by the shift from OM.
	////	const SimTK::Vec3 fM_G = X_GF.R()*fM_F;

	////	// Re-express local vectors in the Ground frame.
	////	SimTK::Vec3 p_FM_G = X_GF.R()  * X_FM.p();    // 15 flops

	////	SpatialVec BushForce_Child = SimTK::SpatialVec(mB2_G, fM_G);
	////	SpatialVec BushForce_Parent = SimTK::SpatialVec(-(mB2_G + p_FM_G % fM_G), -fM_G);




	////	//SpatialVec BushForce_Parent;
	////	//BushForce_Parent[0] = Vec3(F_GF[0][0], F_GF[0][1], F_GF[0][2]);
	////	//BushForce_Parent[1] = Vec3(F_GF[1][0], F_GF[1][1], F_GF[1][2]);
	////	//SpatialVec BushForce_Child;
	////	//BushForce_Child[0] = Vec3(F_GM[0][0], F_GM[0][1], F_GM[0][2]);
	////	//BushForce_Child[1] = Vec3(F_GM[1][0],F_GM[1][1], F_GM[1][2]);







	////	Vec3 FrameLoc1;
	////	FrameLoc1 = EBBEi->getFrame1().findTransformInBaseFrame().T();
	////	std::cout << FrameLoc1 << std::endl;

	////	model->getMatterSubsystem().addInStationForce(*state, EBBEi->getFrame1().getMobilizedBodyIndex(),
	////		FrameLoc1, BushForce_Parent[1], appliedBodyForces);
	////	model->getMatterSubsystem().addInBodyTorque(*state, EBBEi->getFrame1().getMobilizedBodyIndex(),
	////		BushForce_Parent[0], appliedBodyForces);

	////	Vec3 FrameLoc2;
	////	FrameLoc2 = EBBEi->getFrame2().findTransformInBaseFrame().T();
	////	model->getMatterSubsystem().addInStationForce(*state, EBBEi->getFrame2().getMobilizedBodyIndex(),
	////		FrameLoc2, BushForce_Child[1], appliedBodyForces);
	////	model->getMatterSubsystem().addInBodyTorque(*state, EBBEi->getFrame2().getMobilizedBodyIndex(),
	////		BushForce_Child[0], appliedBodyForces);

	////	////int MobInd1 = model->getBodySet().get(EBBEi->getFrame1().findBaseFrame().getName()).getMobilizedBodyIndex();
	////	////int MobInd2 = model->getBodySet().get(EBBEi->getFrame2().findBaseFrame().getName()).getMobilizedBodyIndex();
	////	////appliedBodyForces[MobInd1] = appliedBodyForces[MobInd1] + BushForce_Parent;
	////	////appliedBodyForces[MobInd2] = appliedBodyForces[MobInd2] + BushForce_Child;

	////}

	//////////////////////////////////////////////////////////////////////////////////



	///// knownUdot
	//Vector knownUdot(ndofr);
	////knownUdot.setToZero();
	//for (int i = 0; i < ndofr; ++i) {
	//	knownUdot[i] = ua[i];
	//}
	//// Residual forces
	//Vector residualMobilityForces(ndofr);
	//residualMobilityForces.setToZero();
	//model->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,
	//	appliedMobilityForces, appliedBodyForces, knownUdot,
	//	residualMobilityForces);




	//int nc = 3; // # components in Vec3
	//if (res[0]) {
	//	for (int i = 0; i < ndofr; ++i) {
	//		res[0][i] = value<T>(residualMobilityForces[idx_states2[i]]); // residual torques
	//	}

	//	int j = 0;
	//	for (int iMkr = 0; iMkr < nMkr; ++iMkr) {
	//		for (int k = 0; k < 3; ++k) {
	//			res[0][ndofr + j] = value<T>(MkrGlobalDataMat[iMkr][k]);
	//			++j;
	//		}
	//	}
	//}
	res[0][0] = 1;
	return 0;

}


int main() {
	Recorder x[NX];
	Recorder u[NU];
	Recorder ebbei[nEBBEw];
	Recorder outVar[NR];

	for (int i = 0; i < NX; ++i) x[i] <<= 0;
	for (int i = 0; i < NU; ++i) u[i] <<= 0;
	for (int i = 0; i < nEBBEw; ++i) ebbei[i] <<= 0;

	const Recorder* Recorder_arg[n_in] = { x,u,ebbei };
	Recorder* Recorder_res[n_out] = { outVar };


	F_generic<Recorder>(Recorder_arg, Recorder_res);

	double res[NR];
	for (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];

	Recorder::stop_recording();

	return 0;

}