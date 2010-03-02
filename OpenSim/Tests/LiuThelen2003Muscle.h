#ifndef __LiuThelen2003Muscle_h__
#define __LiuThelen2003Muscle_h__

// LiuThelen2003Muscle.h
// Authors: Peter Loan, Darryl Thelen
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


// INCLUDE
#include <OpenSim/Actuators/Thelen2003Muscle.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A Thelen2003Muscle that includes two states for modeling fatigue and
 * recovery of muscle fibers. The equations for these states are based
 * on the following paper:
 * Liu, Jing Z., Brown, Robert, Yue, Guang H., "A Dynamical Model of Muscle
 * Activation, Fatigue, and Recovery," Biophysical Journal, Vol. 82, Issue 5,
 * pp. 2344-2359, 2002.
 *
 * @author Peter Loan (based on Thelen2003Muscle by Darryl Thelen)
 * @version 1.0
 */
class LiuThelen2003Muscle : public Thelen2003Muscle  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Maximum isometric force that the fibers can generate */
	PropertyDbl _fatigueFactorProp;
	double &_fatigueFactor;

	/** Optimal length of the muscle fibers */
	PropertyDbl _recoveryFactorProp;
	double &_recoveryFactor;

	// Defaults for state variables.
	double _defaultActiveMotorUnits;
	double _defaultFatiguedMotorUnits;

private:
	static const int STATE_ACTIVATION;
	static const int STATE_FIBER_LENGTH;
	static const int STATE_ACTIVE_MOTOR_UNITS;
	static const int STATE_FATIGUED_MOTOR_UNITS;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	LiuThelen2003Muscle();
	LiuThelen2003Muscle(const std::string &aName, double aMaxIsometricForce, double aOptimalFiberLength,
		double aTendonSlackLength, double aPennationAngle, double aFatigueFactor, double aRecoveryFactor);
	LiuThelen2003Muscle(const LiuThelen2003Muscle &aMuscle);
	virtual ~LiuThelen2003Muscle();
	virtual Object* copy() const;

	// Defaults
	virtual double getDefaultActiveMotorUnits() const;
	virtual void setDefaultActiveMotorUnits(double activeMotorUnits);
	virtual double getDefaultFatiguedMotorUnits() const;
	virtual void setDefaultFatiguedMotorUnits(double fatiguedMotorUnits);

#ifndef SWIG
	LiuThelen2003Muscle& operator=(const LiuThelen2003Muscle &aMuscle);
#endif
	void copyData(const LiuThelen2003Muscle &aMuscle);
	virtual void copyPropertyValues(Actuator& aActuator);
#ifndef SWIG
	virtual void initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model);
#endif

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	virtual double getFatigueFactor() const { return _fatigueFactor; }
	virtual bool setFatigueFactor(double aFatigueFactor);
	virtual double getRecoveryFactor() const { return _recoveryFactor; }
	virtual bool setRecoveryFactor(double aRecoveryFactor);

	// Computed quantities
#ifndef SWIG
	virtual double getActiveMotorUnits(const SimTK::State& s) const { return getStateVariable(s, STATE_ACTIVE_MOTOR_UNITS); }
	virtual void setActiveMotorUnits(SimTK::State& s, double activeMotorUnits) const { setStateVariable(s, STATE_ACTIVE_MOTOR_UNITS, activeMotorUnits); }
	virtual double getActiveMotorUnitsDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, STATE_ACTIVE_MOTOR_UNITS); }
	virtual void setActiveMotorUnitsDeriv(const SimTK::State& s, double activeMotorUnitsDeriv) const { setStateVariableDeriv(s, STATE_ACTIVE_MOTOR_UNITS, activeMotorUnitsDeriv); }

	virtual double getFatiguedMotorUnits(const SimTK::State& s) const { return getStateVariable(s, STATE_FATIGUED_MOTOR_UNITS); }
	virtual void setFatiguedMotorUnits(SimTK::State& s, double fatiguedMotorUnits) const { setStateVariable(s, STATE_FATIGUED_MOTOR_UNITS, fatiguedMotorUnits); }
	virtual double getFatiguedMotorUnitsDeriv(const SimTK::State& s) const { return getStateVariableDeriv(s, STATE_FATIGUED_MOTOR_UNITS); }
	virtual void setFatiguedMotorUnitsDeriv(const SimTK::State& s, double fatiguedMotorUnitsDeriv) const { setStateVariableDeriv(s, STATE_FATIGUED_MOTOR_UNITS, fatiguedMotorUnitsDeriv); }

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeStateDerivatives(const SimTK::State& s);
	virtual void computeEquilibrium(SimTK::State& s ) const;
	virtual double computeActuation(const SimTK::State& s) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;
	virtual double computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation);
	virtual void equilibrate(SimTK::State& state) const;

#endif
	virtual void setup(Model& aModel);
	OPENSIM_DECLARE_DERIVED(LiuThelen2003Muscle, Actuator);

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class LiuThelen2003Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __LiuThelen2003Muscle_h__