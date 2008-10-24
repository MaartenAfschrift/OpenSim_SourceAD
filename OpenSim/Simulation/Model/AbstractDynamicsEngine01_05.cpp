// AbstractDynamicsEngine01_05.cpp
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
/**
 * This class used to be the AbstyractDynamcisEngine used as a base class for all DynamicsEngines until 1.6
 * where it was renamed to AbstyractDynamcisEngine01_05 to be used by the versioning code as well as 
 * by dynamics engines other than SimbodyEngine.
 */
//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/Storage.h>
#include "AbstractDynamicsEngine01_05.h"
#include "Model.h"
#include "AbstractBody.h"
#include "AbstractCoordinate.h"
#include "AbstractJoint.h"
#include "CoordinateSet.h"
#include "MarkerSet.h"
#include "SpeedSet.h"
#include "MarkerSet.h"
#include "CoordinateSet.h"
#include "JointSet.h"
#include "BodySet.h"

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractDynamicsEngine01_05::~AbstractDynamicsEngine01_05()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractDynamicsEngine01_05::AbstractDynamicsEngine01_05() :
	Object(),
   _gravity(_gravityProp.getValueDblVec3()),
	_bodySetProp(PropertyObj("", BodySet())),
	_bodySet((BodySet&)_bodySetProp.getValueObj()),
	_jointSetProp(PropertyObj("", JointSet())),
	_jointSet((JointSet&)_jointSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_speedSetProp(PropertyObj("", SpeedSet())),
	_speedSet((SpeedSet&)_speedSetProp.getValueObj()),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
AbstractDynamicsEngine01_05::AbstractDynamicsEngine01_05(const string &aFileName, bool aUpdateFromXMLNode) :
	Object(aFileName, false),
   _gravity(_gravityProp.getValueDblVec3()),
	_bodySetProp(PropertyObj("", BodySet())),
	_bodySet((BodySet&)_bodySetProp.getValueObj()),
	_jointSetProp(PropertyObj("", JointSet())),
	_jointSet((JointSet&)_jointSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_speedSetProp(PropertyObj("", SpeedSet())),
	_speedSet((SpeedSet&)_speedSetProp.getValueObj()),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj())
{
	setNull();
	setupProperties();
	if(aUpdateFromXMLNode) updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 */
AbstractDynamicsEngine01_05::AbstractDynamicsEngine01_05(const AbstractDynamicsEngine01_05& aDE) :
	Object(aDE),
   _gravity(_gravityProp.getValueDblVec3()),
	_bodySetProp(PropertyObj("", BodySet())),
	_bodySet((BodySet&)_bodySetProp.getValueObj()),
	_jointSetProp(PropertyObj("", JointSet())),
	_jointSet((JointSet&)_jointSetProp.getValueObj()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_speedSetProp(PropertyObj("", SpeedSet())),
	_speedSet((SpeedSet&)_speedSetProp.getValueObj()),
	_markerSetProp(PropertyObj("", MarkerSet())),
	_markerSet((MarkerSet&)_markerSetProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aDE);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one AbstractDynamicsEngine01_05 to another.
 *
 * @param aEngine AbstractDynamicsEngine01_05 to be copied.
 */
void AbstractDynamicsEngine01_05::copyData(const AbstractDynamicsEngine01_05& aEngine)
{
	_gravity = aEngine._gravity;
	_model = aEngine._model;
	_bodySet = aEngine._bodySet;
	_coordinateSet = aEngine._coordinateSet;
	_speedSet = aEngine._speedSet;
	_jointSet = aEngine._jointSet;
	_markerSet = aEngine._markerSet;
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void AbstractDynamicsEngine01_05::setNull()
{
	setType("AbstractDynamicsEngine01_05");

	_model = NULL;
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel model containing this dynamics engine.
 * 
 * Moved setup calls inside the sets to handle groups properly. -Ayman 6/07
 */
void AbstractDynamicsEngine01_05::setup(Model* aModel)
{
	_model = aModel;
/*
	_bodySet.setup(this);
	_coordinateSet.setup(this);
	_speedSet.setup(this);
	_jointSet.setup(this);
	_markerSet.setup(this);*/
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 *
 */
void AbstractDynamicsEngine01_05::setupProperties()
{
	const SimTK::Vec3 defaultGravity(0.0, -9.80665, 0.0);
	_gravityProp.setComment("Acceleration due to gravity.");
	_gravityProp.setName("gravity");
	_gravityProp.setValue(defaultGravity);
	//_gravityProp.setAllowableArraySize(3);
	_propertySet.append(&_gravityProp);

	// Note: PropertyObj tag names come from the object's type (e.g. _bodySetProp below will automatically be associated with <BodySet> tag)
	// don't need to call _bodySetProp.setName()...
	_bodySetProp.setComment("Bodies in the model.");
	_propertySet.append(&_bodySetProp);

	_jointSetProp.setComment("Joints in the model.");
	_propertySet.append(&_jointSetProp);

	_coordinateSetProp.setComment("Generalized coordinates in the model.");
	_propertySet.append(&_coordinateSetProp);

	_speedSetProp.setComment("Generalized speeds in the model.");
	_propertySet.append(&_speedSetProp);

	_markerSetProp.setComment("Markers in the model.");
	_propertySet.append(&_markerSetProp);
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
AbstractDynamicsEngine01_05& AbstractDynamicsEngine01_05::operator=(const AbstractDynamicsEngine01_05 &aDE)
{
	// Base class
	Object::operator=(aDE);

	copyData(aDE);

	return(*this);
}

//--------------------------------------------------------------------------
// SCALING
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Scale the dynamics engine
 *
 * @param aScaleSet the set of XYZ scale factors for the bodies
 * @param aFinalMass the mass that the scaled model should have
 * @param aPreserveMassDist whether or not the masses of the
 *        individual bodies should be scaled with the body scale factors.
 * @return Whether or not scaling was successful.
 */
bool AbstractDynamicsEngine01_05::scale(const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
	_bodySet.scale(aScaleSet, !aPreserveMassDist);

	// Now that the masses of the individual bodies have
	// been scaled (if aPreserveMassDist == false), get the
	// total mass and compare it to aFinalMass in order to
	// determine how much to scale the body masses again,
	// so that the total model mass comes out to aFinalMass.
	if (aFinalMass > 0.0)
	{
		double mass = getMass();
		if (mass > 0.0)
		{
			double factor = aFinalMass / mass;
			for (int i = 0; i < _bodySet.getSize(); i++)
				_bodySet.get(i)->scaleMass(factor);
		}
	}

	// Now scale the joints.
	_jointSet.scale(aScaleSet);

	// Now scale the markers.
	_markerSet.scale(aScaleSet);

	return true;
}

//=============================================================================
// NUMBERS
//=============================================================================
int AbstractDynamicsEngine01_05::getNumBodies() const { return _bodySet.getSize(); }
int AbstractDynamicsEngine01_05::getNumJoints() const { return _jointSet.getSize(); }
int AbstractDynamicsEngine01_05::getNumCoordinates() const { return _coordinateSet.getSize(); }
int AbstractDynamicsEngine01_05::getNumSpeeds() const { return _speedSet.getSize(); }
int AbstractDynamicsEngine01_05::getNumMarkers() const { return _markerSet.getSize(); }

//--------------------------------------------------------------------------
// GRAVITY
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the gravity vector in the gloabl frame.
 *
 * @param rGrav the XYZ gravity vector in the global frame is returned here.
 */
void AbstractDynamicsEngine01_05::getGravity(SimTK::Vec3& rGrav) const
{
	rGrav = _gravity;
}

//_____________________________________________________________________________
/**
 * Set the gravity vector in the gloabl frame.
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
bool AbstractDynamicsEngine01_05::setGravity(const SimTK::Vec3& aGrav)
{
	_gravity = aGrav;

	return true;
}

//=============================================================================
// BODY INFORMATION
//=============================================================================
AbstractWrapObject* AbstractDynamicsEngine01_05::getWrapObject(const string& aName) const
{
	for (int i = 0; i < _bodySet.getSize(); i++) {
		AbstractWrapObject* wrap = _bodySet.get(i)->getWrapObject(aName);
		if (wrap != NULL)
			return wrap;
	}

	return NULL;
}

//--------------------------------------------------------------------------
// MARKERS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Write an XML file of all the markers in the model.
 *
 * @param aFileName the name of the file to create
 */
void AbstractDynamicsEngine01_05::writeMarkerFile(const string& aFileName) const
{
	_markerSet.print(aFileName);
}

//_____________________________________________________________________________
/**
 * Replace all markers in the model with the ones in the passed-in marker set.
 *
 * @param aMarkerSet The new marker set.
 * @return Number of markers that were successfully added to the model.
 */
int AbstractDynamicsEngine01_05::replaceMarkerSet(MarkerSet& aMarkerSet)
{

	int i, numAdded = 0;
	/*
	// First remove all existing markers from the model.
	_markerSet.clearAndDestroy();
	_markerSetProp.setUseDefault(false);

	// Now add the markers from aMarkerSet whose body names match bodies in the engine.
	for (i = 0; i < aMarkerSet.getSize(); i++)
	{
		// Eran: we make a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
		AbstractMarker* marker = (AbstractMarker*)aMarkerSet.get(i)->copy();
		const string* bodyName = marker->getBodyName();
		AbstractBody* body = _bodySet.get(*bodyName);
		if (body)
		{
			marker->setBody(*body, false);
			_markerSet.append(marker);
			numAdded++;
		}
	}

	cout << "Replaced marker set in model " << _model->getName() << endl; */
	return numAdded;
}

//_____________________________________________________________________________
/**
 * Update all markers in the model with the ones in the
 * passed-in marker set. If the marker does not yet exist
 * in the model, it is added.
 *
 * @param aMarkerSet set of markers to be updated/added
 */
void AbstractDynamicsEngine01_05::updateMarkerSet(MarkerSet& aMarkerSet)
{
	_markerSetProp.setUseDefault(false);
	for (int i = 0; i < aMarkerSet.getSize(); i++)
	{
		AbstractMarker* updatingMarker = aMarkerSet.get(i);
		AbstractMarker* modelMarker = getMarkerSet()->get(updatingMarker->getName());
		const string* updatingBodyName = updatingMarker->getBodyName();

		/* If there is already a marker in the model with that name,
		 * update it with the parameters from the updating marker,
		 * moving it to a new body if necessary.
		 */
		if (modelMarker)
		{
			/* If the updating marker is on a different body, delete the
			 * marker from the model and add the updating one (as long as
			 * the updating marker's body exists in the model).
			 */
			if (modelMarker->getBody() && updatingBodyName &&
				 modelMarker->getBody()->getName() != *updatingBodyName)
			{
				_markerSet.remove(modelMarker);
				// Eran: we append a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
				_markerSet.append((AbstractMarker*)updatingMarker->copy());
			}
			else
			{
				modelMarker->updateFromMarker(*updatingMarker);
			}
		}
		else
		{
			/* The model does not contain a marker by that name. If it has
			 * a body by that name, add the updating marker to the markerset.
			 */
			// Eran: we append a *copy* since both _markerSet and aMarkerSet own their elements (so they will delete them)
			if (updatingBodyName && _bodySet.get(*updatingBodyName))
				_markerSet.append((AbstractMarker*)updatingMarker->copy());
		}
	}

	// TODO: We need to call setup again to make sure the _body pointers are up to date; but
	// note that we've already called setup before so we need to make sure the setup() function
	// supports getting called multiple times
	/*
	for (int i = 0; i < _markerSet.getSize(); i++)
		_markerSet.get(i)->setup(this);
	*/
	cout << "Updated markers in model " << _model->getName() << endl;
}

//_____________________________________________________________________________
/**
 * Remove all markers from the model that are not in the passed-in list.
 *
 * @param aMarkerNames array of marker names not to be deleted
 * @return Number of markers deleted
 *
 * @TODO_AYMAN make sure visuals adjust as well
 */
int AbstractDynamicsEngine01_05::deleteUnusedMarkers(const Array<string>& aMarkerNames)
{
	int i, numDeleted = 0;

	for (i = 0; i < _markerSet.getSize(); )
	{
		int index = aMarkerNames.findIndex(_markerSet.get(i)->getName());
		if (index < 0)
		{
			// Delete the marker, but don't increment i or else you'll
			// skip over the marker right after the deleted one.
			_markerSet.get(i)->removeSelfFromDisplay();
			_markerSet.remove(i);
			numDeleted++;
		}
		else
		{
			i++;
		}
	}

	cout << "Deleted " << numDeleted << " unused markers from model " << _model->getName() << endl;

	return numDeleted;
}


//=============================================================================
// CONFIGURATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Extract the configuration (coordinates and speeds) of a model from the states.
 *
 * @param aYStore Storage object containing a complete set of model states.
 * @param rQStore Storage object containing the coordinates held in aYStore.
 * @param rUStore Storage object containing the speeds held in aYStore.
 */
void AbstractDynamicsEngine01_05::
extractConfiguration(const Storage &aYStore,Storage &rQStore,Storage &rUStore)
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	int nqnu = nq+nu;

	// CONFIGURE
	// Names
	rQStore.setName("Coordinates");
	rUStore.setName("Speeds");
	// Description
	rQStore.setDescription(aYStore.getDescription());
	rUStore.setDescription(aYStore.getDescription());
	// Column labels
	Array<string> qLabels("",nq),uLabels("",nu);
	Array<string> labels = aYStore.getColumnLabels();
	for(int i=0;i<nq;i++) qLabels[i] = labels[i];
	for(int i=0;i<nu;i++) uLabels[i] = labels[nq+i];
	rQStore.setColumnLabels(qLabels);
	rUStore.setColumnLabels(uLabels);
	// Purge
	rQStore.purge();
	rUStore.purge();

	// LOOP THROUGH STATES
	int size = aYStore.getSize();
	StateVector *vector;
	double time;
	Array<double> data(0.0);
	Array<double> q(0.0,nq);
	for(int i=0;i<size;i++) {
		vector = aYStore.getStateVector(i);
		if(vector->getSize()<nqnu) continue;
		time = vector->getTime();
		data = vector->getData();
		rQStore.append(time,nq,&data[0]);
		rUStore.append(time,nu,&data[nq]);
	}
}
//_____________________________________________________________________________
/**
 * From a potentially partial specification of the generalized coordinates,
 * form a complete storage of the generalized coordinates (q's) and
 * generalized speeds (u's).
 *
 * @param aQIn Storage containing the q's or a subset of the q's.  Rotational
 * q's should be in degrees.
 * @param rQComplete Storage containing all the q's.  If q's were not
 * in aQIn, the values are set to 0.0.  When a q is constrained, its value
 * is altered to be consistent with the constraint.  The caller is responsible
 * for deleting the memory associated with this storage.
 * @param rUComplete Storage containing all the u's.  The generalized speeds
 * are obtained by spline fitting the q's and differentiating the splines.
 * When a u is constrained, its value is altered to be consisten with the
 * constraint.  The caller is responsible for deleting the memory
 * associated with this storage.
 */
void AbstractDynamicsEngine01_05::
formCompleteStorages(const OpenSim::Storage &aQIn,
	OpenSim::Storage *&rQComplete,OpenSim::Storage *&rUComplete) const
{
	int i;
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();

	// Get coordinate file indices
	Array<string> columnLabels;
	columnLabels.append("time");
	string qName;
	AbstractCoordinate *coord;
	Array<int> index(-1,nq);
	const CoordinateSet *coordinateSet = getCoordinateSet();
	int sizeCoordSet = coordinateSet->getSize();
	for(i=0;i<sizeCoordSet;i++) {
		coord = coordinateSet->get(i);
		if(coord==NULL) continue;
		qName = coord->getName();
		columnLabels.append(qName);
		index[i] = aQIn.getStateIndex(qName);
		if(index[i]<0) {
			string msg = "Model::formCompleteStorages(): WARNING- Did not find column ";
			msg += qName;
			msg += " in storage object.\n";
			cout<<msg;
		}
	}

	// Extract Coordinates
	double time;
	Array<double> data(0.0);
	Array<double> q(0.0,nq);
	Storage *qStore = new Storage();
	qStore->setName("GeneralizedCoordinates");
	qStore->setColumnLabels(columnLabels);
	int size = aQIn.getSize();
	StateVector *vector;
	int j;
	for(i=0;i<size;i++) {
		vector = aQIn.getStateVector(i);
		data = vector->getData();
		time = vector->getTime();

		for(j=0;j<nq;j++) {
			q[j] = 0.0;
			if(index[j]<0) continue;
			q[j] = data[index[j]];
		}

		qStore->append(time,nq,&q[0]);
	}

	// Convert to radians
	convertDegreesToRadians(*qStore);

	// Compute generalized speeds
	GCVSplineSet tempQset(5,qStore);
	Storage *uStore = tempQset.constructStorage(1);

	// Compute constraints
	Array<double> qu(0.0,nq+nu);
	//int qSize = qStore->getSize();
	//int uSize = uStore->getSize();
	//cout<<"qSize = "<<qSize<<endl<<endl;
	//cout<<"uSize = "<<uSize<<endl<<endl;
	rQComplete = new Storage();
	rUComplete = new Storage();
	for(i=0;i<size;i++) {
		qStore->getTime(i,time);
		qStore->getData(i,nq,&qu[0]);
		uStore->getData(i,nq,&qu[nq]);
		computeConstrainedCoordinates(&qu[0]);
		rQComplete->append(time,nq,&qu[0]);
		rUComplete->append(time,nu,&qu[nq]);
	}
	
	delete qStore;
	
	// Compute storage object for simulation
	// Need to set column labels before converting rad->deg
	rQComplete->setColumnLabels(columnLabels);
	rUComplete->setColumnLabels(columnLabels);

	// Convert back to degrees
	convertRadiansToDegrees(*rQComplete);
	convertRadiansToDegrees(*rUComplete);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 */
void AbstractDynamicsEngine01_05::scaleRotationalDofColumns(Storage &rStorage, double factor) const
{
	const Array<std::string>& columnLabels = rStorage.getColumnLabels();

	if(columnLabels.getSize() == 0)
		throw Exception("AbstractDynamicsEngine01_05.scaleRotationalDofColumns: ERROR- storage has no labels, can't determine coordinate types for deg<->rad conversion",
							 __FILE__,__LINE__);

	// Loop through the coordinates in the model. For each one that is rotational,
	// see if it has a corresponding column of data. If it does, multiply that
	// column by the given scaling factor.
	const CoordinateSet* coordinateSet = _model->getDynamicsEngine().getCoordinateSet();
	for (int i = 0; i < coordinateSet->getSize(); i++) {
		if (coordinateSet->get(i)->getMotionType() == AbstractTransformAxis::Rotational) {
			std::string name = coordinateSet->get(i)->getName();
			for(int j=1; j<columnLabels.getSize(); j++) // skip time column (and adjust for time column when calling multiplyColumn)
				if(columnLabels[j] == name) rStorage.multiplyColumn(j-1, factor);
		}
	}

	// Now do speeds
	const SpeedSet* speedSet = _model->getDynamicsEngine().getSpeedSet();
	for (int i = 0; i < speedSet->getSize(); i++) {
		if (speedSet->get(i)->getCoordinate() && speedSet->get(i)->getCoordinate()->getMotionType() == AbstractTransformAxis::Rotational) {
			std::string name = speedSet->get(i)->getName();
			assert(name != speedSet->get(i)->getCoordinate()->getName()); // speed should have different name than coordinate (else we'll end up scaling twice)
			for(int j=1; j<columnLabels.getSize(); j++) // skip time column (and adjust for time column when calling multiplyColumn)
				if(columnLabels[j] == name) rStorage.multiplyColumn(j-1, factor);
		}
	}
}
//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * degrees to units of radians for all the state-vectors in an Storage
 * object.  Coordinates/speeds are identified by column names.
 *
 * @param rStorage Storage object.
 */
void AbstractDynamicsEngine01_05::convertDegreesToRadians(Storage &rStorage) const
{
	scaleRotationalDofColumns(rStorage, SimTK_DEGREE_TO_RADIAN);
}
//_____________________________________________________________________________
/**
 * Convert the rotational generalized coordinates or speeds from units of
 * radians to units of degrees for all the state-vectors in an Storage
 * object.  Coordinates/speeds are identified by column names.
 *
 * @param rStorage Storage object.
 */
void AbstractDynamicsEngine01_05::convertRadiansToDegrees(Storage &rStorage) const
{
	scaleRotationalDofColumns(rStorage, SimTK_RADIAN_TO_DEGREE);
}
//_____________________________________________________________________________
/**
 * Convert an array of Q/U values from degrees to radians. The sizes of the
 * arrays are assumed to be equal to the number of Us.
 *
 * @param aQDeg Array of values, in degrees
 * @param rQRad Array of values, in radians
 */
void AbstractDynamicsEngine01_05::convertDegreesToRadians(double *aQDeg, double *rQRad) const
{
	const CoordinateSet* coordinateSet = _model->getDynamicsEngine().getCoordinateSet();

	// The arrays aQDeg and rQRad are assumed to be of size getNumSpeeds() or greater.
	// It is also assumed that aQDeg[N] corresponds to the first N coordinates
	// in the model, whether those N values are coordinates or speeds.
	for (int i = 0; i < getNumSpeeds(); i++)
	{
		if (coordinateSet->get(i)->getMotionType() == AbstractTransformAxis::Rotational)
			rQRad[i] = aQDeg[i] * SimTK_DEGREE_TO_RADIAN;
		else
			rQRad[i] = aQDeg[i];
	}
}
//_____________________________________________________________________________
/**
 * Convert an array of Q/U values from radians to degrees. The sizes of the
 * arrays are assumed to be equal to the number of Us.
 *
 * @param aQRad Array of values, in radians
 * @param rQDeg Array of values, in degrees
 */
void AbstractDynamicsEngine01_05::convertRadiansToDegrees(double *aQRad, double *rQDeg) const
{
	const CoordinateSet* coordinateSet = _model->getDynamicsEngine().getCoordinateSet();

	// The arrays aQRad and rQDeg are assumed to be of size getNumSpeeds() or greater.
	// It is also assumed that aQRad[N] corresponds to the first N coordinates
	// in the model, whether those N values are coordinates or speeds.
	for (int i = 0; i < getNumSpeeds(); i++)
	{
		if (coordinateSet->get(i)->getMotionType() == AbstractTransformAxis::Rotational)
			rQDeg[i] = aQRad[i] * SimTK_RADIAN_TO_DEGREE;
		else
			rQDeg[i] = aQRad[i];
	}
}