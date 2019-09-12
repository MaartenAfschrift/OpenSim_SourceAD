/* -------------------------------------------------------------------------- *
*                       OpenSim:  MomentArmSolver.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Ajay Seth                                                       *
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

#include "MomentArmSolver.h"
#include "Model/PointForceDirection.h"
#include "Model/Model.h"

using namespace std;
using namespace SimTK;

namespace OpenSim {

    //______________________________________________________________________________
    /**
    * An implementation of the MomentArmSolver
    *
    */
    MomentArmSolver::MomentArmSolver(const Model &model) : Solver(model)
    {
        setAuthors("Ajay Seth");
        _stateCopy = model.getWorkingState();

        // Get the body forces equivalent of the point forces of the path
        _bodyForces = getModel().getSystem()
            .getRigidBodyForces(_stateCopy, Stage::Instance);
        // get the right size coupling vector
        _coupling = _stateCopy.getU();
    }

    /*********************************************************************************
    Solve for moment arm, r = d(path_length)/d(theta)

    Ideally we want to compute r without perturbing theta values and recomputing
    lengths because it is inexact, very sensitive to perturbations that may cause a
    path (via) point to appear or disappear, and inefficient to recompute lengths,
    especially for complex paths involving wrapping.

    Refer to Moment-arm Theory document by Michael Sherman for details.

    **********************************************************************************/
    osim_double_adouble MomentArmSolver::solve(const State &state, const Coordinate &aCoord,
        const GeometryPath &path) const
    {
        //Local modifiable copy of the state
        State& s_ma = _stateCopy;
        s_ma.updQ() = state.getQ();


        // compute the coupling between coordinates due to constraints
        _coupling = computeCouplingVector(s_ma, aCoord);

        // set speeds to zero
        s_ma.updU() = 0;

        // zero out all the forces
        _bodyForces *= 0;
        _generalizedForces = 0;

        // apply a tension of unity to the bodies of the path
        Vector pathDependentMobilityForces(s_ma.getNU(), 0.0);
        path.addInEquivalentForces(s_ma, 1.0, _bodyForces, pathDependentMobilityForces);

        //_bodyForces.dump("bodyForces from addInEquivalentForcesOnBodies");

        // Convert body spatial forces F to equivalent mobility forces f based on 
        // geometry (no dynamics required): f = ~J(q) * F.
        getModel().getMultibodySystem().getMatterSubsystem()
            .multiplyBySystemJacobianTranspose(s_ma, _bodyForces, _generalizedForces);

        _generalizedForces += pathDependentMobilityForces;
        // Moment-arm is the effective torque (since tension is 1) at the 
        // coordinate of interest taking into account the generalized forces also 
        // acting on other coordinates that are coupled via constraint.

        //std::cout << _coupling << std::endl;


        return ~_coupling*_generalizedForces;
    }



    osim_double_adouble MomentArmSolver::solve(const State &state, const Coordinate &aCoord,
        const Array<PointForceDirection *> &pfds) const
    {
        //const clock_t start = clock();

        //Local modifiable copy of the state
        State& s_ma = _stateCopy;
        s_ma.updQ() = state.getQ();

        // compute the coupling between coordinates due to constraints
        _coupling = computeCouplingVector(s_ma, aCoord);

        // set speeds to zero
        s_ma.updU() = 0;

        int n = pfds.getSize();
        // Apply body forces along the geometry described by pfds due to a tension of 1N
        for (int i = 0; i<n; i++) {
            getModel().getMatterSubsystem().
                addInStationForce(s_ma,
                    pfds[i]->frame().getMobilizedBodyIndex(),
                    pfds[i]->point(), pfds[i]->direction(), _bodyForces);
        }

        //_bodyForces.dump("bodyForces from PointForceDirections");

        // Convert body spatial forces F to equivalent mobility forces f based on 
        // geometry (no dynamics required): f = ~J(q) * F.
        getModel().getMultibodySystem().getMatterSubsystem()
            .multiplyBySystemJacobianTranspose(s_ma, _bodyForces, _generalizedForces);

        // Moment-arm is the effective torque (since tension is 1) at the 
        // coordinate of interest taking into account the generalized forces also 
        // acting on other coordinates that are coupled via constraint.
        return ~_coupling*_generalizedForces;
    }

    SimTK::Vector MomentArmSolver::computeCouplingVector(SimTK::State &state,
        const Coordinate &coordinate) const
    {
        // make sure copy of the state is realized to at least instance
        getModel().getMultibodySystem().realize(state, SimTK::Stage::Instance);

        // unlock the coordinate if it is locked
        coordinate.setLocked(state, false);

        // Calculate coupling matrix C to determine the influence of other coordinates 
        // (mobilities) on the coordinate of interest due to constraints
        state.updU() = 0;
        // Light-up speed of coordinate of interest and see how other coordinates
        // affected by constraints respond
        coordinate.setSpeedValue(state, 1);
        getModel().getMultibodySystem().realize(state, SimTK::Stage::Velocity);

        // Satisfy all the velocity constraints.
        getModel().getMultibodySystem().projectU(state, 1e-10);

        // Now calculate C. by checking how speeds of other coordinates change
        // normalized by how much the speed of the coordinate of interest changed 
        return state.getU() / coordinate.getSpeedValue(state);
    }

} // end of namespace OpenSim




/////////* -------------------------------------------------------------------------- *
////////*                       OpenSim:  MomentArmSolver.cpp                        *
////////* -------------------------------------------------------------------------- *
////////* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
////////* See http://opensim.stanford.edu and the NOTICE file for more information.  *
////////* OpenSim is developed at Stanford University and supported by the US        *
////////* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
////////* through the Warrior Web program.                                           *
////////*                                                                            *
////////* Copyright (c) 2005-2017 Stanford University and the Authors                *
////////* Author(s): Ajay Seth                                                       *
////////*                                                                            *
////////* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
////////* not use this file except in compliance with the License. You may obtain a  *
////////* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
////////*                                                                            *
////////* Unless required by applicable law or agreed to in writing, software        *
////////* distributed under the License is distributed on an "AS IS" BASIS,          *
////////* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
////////* See the License for the specific language governing permissions and        *
////////* limitations under the License.                                             *
////////* -------------------------------------------------------------------------- */
////////
////////#include "MomentArmSolver.h"
////////#include "Model/PointForceDirection.h"
////////#include "Model/Model.h"
////////#include <fstream>
////////
////////
////////using namespace std;
////////using namespace SimTK;
////////
////////namespace OpenSim {
////////
////////	//______________________________________________________________________________
////////	/**
////////	* An implementation of the MomentArmSolver
////////	*
////////	*/
////////	MomentArmSolver::MomentArmSolver(const Model &model) : Solver(model)
////////	{
////////		setAuthors("Ajay Seth");
////////		//_stateCopy = model.getWorkingState();
////////
////////		// Get the body forces equivalent of the point forces of the path
////////		//_bodyForces = getModel().getSystem()
////////		//       .getRigidBodyForces(_stateCopy, Stage::Instance);
////////		// get the right size coupling vector
////////		//_coupling = SimTK::Vector(_stateCopy.getNU()); /// this line was adapted to work with AD
////////	}
////////
////////	/*********************************************************************************
////////	Solve for moment arm, r = d(path_length)/d(theta)
////////
////////	Ideally we want to compute r without perturbing theta values and recomputing
////////	lengths because it is inexact, very sensitive to perturbations that may cause a
////////	path (via) point to appear or disappear, and inefficient to recompute lengths,
////////	especially for complex paths involving wrapping.
////////
////////	Refer to Moment-arm Theory document by Michael Sherman for details.
////////
////////	**********************************************************************************/
////////	//osim_double_adouble MomentArmSolver::solve(const State &state, const Coordinate &aCoord,
////////	//                              const GeometryPath &path, const Model& model) const
////////	osim_double_adouble MomentArmSolver::solve(const State &state, const Coordinate &aCoord,
////////		const GeometryPath &path) const
////////	{
////////		SimTK::Vector_<SimTK::SpatialVec> _bodyForces = getModel().getSystem()
////////			.getRigidBodyForces(state, Stage::Instance);
////////
////////		//Local modifiable copy of the state
////////		State* s_ma = new State(state);
////////		s_ma->updQ() = state.getQ();
////////
////////		// compute the coupling between coordinates due to constraints
////////		//_coupling = computeCouplingVector(s_ma, aCoord);
////////		//computeCouplingVector(s_ma, aCoord, _coupling);
////////		//couplingVector = computeCouplingVector(s_ma, aCoord);
////////
////////		// make sure copy of the state is realized to at least instance
////////		getModel().getMultibodySystem().realize(*s_ma, SimTK::Stage::Instance);
////////
////////		// unlock the coordinate if it is locked
////////		aCoord.setLocked(*s_ma, false);
////////
////////		// Calculate coupling matrix C to determine the influence of other coordinates 
////////		// (mobilities) on the coordinate of interest due to constraints
////////
////////		//s_ma->updU() = 0;
////////		//Trick to make things working with adoubles
////////		adouble tempp = 0.0;
////////		Vector nulvector(s_ma->getNU());
////////		for (int i = 0; i < s_ma->getNU(); i++)
////////		{
////////			nulvector[i] = tempp;
////////		}
////////		s_ma->setU(nulvector);
////////
////////
////////		// Light-up speed of coordinate of interest and see how other coordinates
////////		// affected by constraints respond
////////		osim_double_adouble tempvalue = 1.0;
////////		aCoord.setSpeedValue(*s_ma, tempvalue);
////////		getModel().getMultibodySystem().realize(*s_ma, SimTK::Stage::Velocity);
////////
////////		// Satisfy all the velocity constraints.
////////		//osim_double_adouble tempvalue2 = 1e-10;
////////		//getModel().getMultibodySystem().projectU(state, tempvalue2);
////////		//getModel().getMultibodySystem().projectU(state, 1e-10);
////////
////////		// Now calculate C. by checking how speeds of other coordinates change
////////		// normalized by how much the speed of the coordinate of interest changed 
////////
////////		//Vector couplingVector(state.getNU());
////////		Vector couplingVector(s_ma->getNU());
////////
////////		for (int i = 0; i < s_ma->getNU(); ++i) {
////////			couplingVector[i] = s_ma->getU()[i] / aCoord.getSpeedValue(*s_ma);
////////		}
////////
////////		// set speeds to zero
////////		s_ma->updU().setToZero();
////////		//adouble tempp2 = 0.0;
////////		//Vector nulvector2(s_ma->getNU());
////////		//for (int i = 0; i < s_ma->getNU(); i++)
////////		//{
////////		//	nulvector2[i] = tempp2;
////////		//}
////////		//s_ma->setU(nulvector2);
////////
////////		// zero out all the forces
////////		_bodyForces *= 0; // causing issue => loop ???
////////
////////		//std::ofstream myfile3aa;
////////		//myfile3aa.open("test3aa.txt", std::ofstream::app);
////////		//myfile3aa << _bodyForces; myfile3aa << "  ";
////////		//myfile3aa.close();
////////
////////		//std::ofstream myfile3a;
////////		//myfile3a.open("test3a.txt", std::ofstream::app);
////////		//myfile3a << state.getQ(); myfile3a << "  ";
////////		//myfile3a << state.getU(); myfile3a << "  " << std::endl;
////////		//myfile3a.close();
////////
////////		//adouble tempp3 = 0.0;
////////		//for (int i = 0; i < _bodyForces.size(); i++) {
////////		//	for (int j = 0; j < 2; j++) {
////////		//		for (int k = 0; k < 3; k++) {
////////		//			_bodyForces[i][j][k] = tempp3;
////////		//		}
////////		//	}
////////		//}
////////
////////		//std::ofstream myfile3ab;
////////		//myfile3ab.open("test3ab.txt", std::ofstream::app);
////////		//myfile3ab << _bodyForces; myfile3ab << "  ";
////////		//myfile3ab.close();
////////
////////		//std::ofstream myfile3;
////////		//myfile3.open("test3.txt", std::ofstream::app);
////////		//myfile3 << state.getQ(); myfile3 << "  ";
////////		//myfile3 << state.getU(); myfile3 << "  " << std::endl;
////////		//myfile3.close();
////////
////////		SimTK::Vector _generalizedForces;
////////		_generalizedForces = 0;
////////		//for (int i = 0; i < _generalizedForces.size(); i++) {
////////		//		_generalizedForces[i] = 0;
////////		//}
////////
////////		// apply a tension of unity to the bodies of the path
////////		Vector pathDependentMobilityForces(s_ma->getNU(), 0.0);
////////
////////		//std::ofstream myfile4;
////////		//myfile4.open("test4.txt", std::ofstream::app);
////////		//myfile4 << state.getQ(); myfile4 << "  ";
////////		//myfile4 << state.getU(); myfile4 << "  " << std::endl;
////////		//myfile4.close();
////////
////////		osim_double_adouble temp_tension = 1.0;
////////		path.addInEquivalentForces(*s_ma, temp_tension, _bodyForces, pathDependentMobilityForces);
////////
////////		//std::ofstream myfile5;
////////		//myfile5.open("test5.txt", std::ofstream::app);
////////		//myfile5 << state.getQ(); myfile5 << "  ";
////////		//myfile5 << state.getU(); myfile5 << "  " << std::endl;
////////		//myfile5.close();
////////
////////		//_bodyForces.dump("bodyForces from addInEquivalentForcesOnBodies");
////////
////////		// Convert body spatial forces F to equivalent mobility forces f based on 
////////		// geometry (no dynamics required): f = ~J(q) * F.
////////		getModel().getMultibodySystem().getMatterSubsystem()
////////			.multiplyBySystemJacobianTranspose(*s_ma, _bodyForces, _generalizedForces);
////////
////////		_generalizedForces += pathDependentMobilityForces;
////////
////////		// Moment-arm is the effective torque (since tension is 1) at the 
////////		// coordinate of interest taking into account the generalized forces also 
////////		// acting on other coordinates that are coupled via constraint.
////////
////////		//std::ofstream myfile6;
////////		//myfile6.open("test6.txt", std::ofstream::app);
////////		//myfile6 << state.getQ(); myfile6 << "  ";
////////		//myfile6 << state.getU(); myfile6 << "  " << std::endl;
////////		//myfile6.close();
////////
////////		return ~couplingVector*_generalizedForces;
////////
////////	}
////////
////////	//osim_double_adouble MomentArmSolver::solve(const State &state, const Coordinate &aCoord,
////////	//                              const Array<PointForceDirection *> &pfds) const
////////	//{
////////	//    //const clock_t start = clock();
////////	//
////////	//    //Local modifiable copy of the state
////////	//    State& s_ma = _stateCopy;
////////	//    s_ma.updQ() = state.getQ();
////////	//
////////	//    // compute the coupling between coordinates due to constraints
////////	//    //_coupling = computeCouplingVector(s_ma, aCoord);
////////	//	Vector _coupling(_stateCopy.getNU());
////////	//	_coupling = computeCouplingVector(s_ma, aCoord);
////////	//
////////	//    // set speeds to zero
////////	//    s_ma.updU() = 0;
////////	//
////////	//    int n = pfds.getSize();
////////	//    // Apply body forces along the geometry described by pfds due to a tension of 1N
////////	//    for(int i=0; i<n; i++) {
////////	//        getModel().getMatterSubsystem().
////////	//            addInStationForce(s_ma, 
////////	//                pfds[i]->frame().getMobilizedBodyIndex(), 
////////	//                pfds[i]->point(), pfds[i]->direction(), _bodyForces);
////////	//    }
////////	//
////////	//    //_bodyForces.dump("bodyForces from PointForceDirections");
////////	//
////////	//    // Convert body spatial forces F to equivalent mobility forces f based on 
////////	//    // geometry (no dynamics required): f = ~J(q) * F.
////////	//    getModel().getMultibodySystem().getMatterSubsystem()
////////	//        .multiplyBySystemJacobianTranspose(s_ma, _bodyForces, _generalizedForces);
////////	//
////////	//    // Moment-arm is the effective torque (since tension is 1) at the 
////////	//    // coordinate of interest taking into account the generalized forces also 
////////	//    // acting on other coordinates that are coupled via constraint.
////////	//    return ~_coupling*_generalizedForces;
////////	//}
////////
////////	//SimTK::Vector MomentArmSolver::computeCouplingVector(SimTK::State &state,
////////	//	const Coordinate &coordinate) const
////////	//	//void MomentArmSolver::computeCouplingVector(SimTK::State &state,
////////	//	//	const Coordinate &coordinate, Vector &couplingVector)  const
////////	//	//void MomentArmSolver::computeCouplingVector(SimTK::State &state,
////////	//	//	const Coordinate &coordinate, Vector &_coupling)  const
////////	//{
////////	//	// make sure copy of the state is realized to at least instance
////////	//	getModel().getMultibodySystem().realize(state, SimTK::Stage::Instance);
////////
////////	//	// unlock the coordinate if it is locked
////////	//	coordinate.setLocked(state, false);
////////
////////	//	// Calculate coupling matrix C to determine the influence of other coordinates 
////////	//	// (mobilities) on the coordinate of interest due to constraints
////////	//	state.updU() = 0;
////////	//	// Light-up speed of coordinate of interest and see how other coordinates
////////	//	// affected by constraints respond
////////
////////	//	osim_double_adouble tempvalue = 1.0;
////////	//	coordinate.setSpeedValue(state, tempvalue);
////////	//	getModel().getMultibodySystem().realize(state, SimTK::Stage::Velocity);
////////
////////	//	// Satisfy all the velocity constraints.
////////
////////	//	//osim_double_adouble tempvalue2 = 1e-10;
////////	//	//getModel().getMultibodySystem().projectU(state, tempvalue2);
////////	//	//getModel().getMultibodySystem().projectU(state, 1e-10);
////////
////////	//	// Now calculate C. by checking how speeds of other coordinates change
////////	//	// normalized by how much the speed of the coordinate of interest changed 
////////
////////	//	//Vector couplingVector(state.getNU());
////////	//	Vector _coupling(state.getNU());
////////
////////	//	for (int i = 0; i < state.getNU(); ++i) {
////////	//		_coupling[i] = state.getU()[i] / coordinate.getSpeedValue(state);
////////	//	}
////////
////////	//	std::cout << "computeCouplingVector" << _coupling << std::endl;
////////	//	return _coupling;
////////
////////	//}
////////
////////} // end of namespace opensim