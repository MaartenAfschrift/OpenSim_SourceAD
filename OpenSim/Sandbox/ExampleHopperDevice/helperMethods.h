#ifndef _helperMethods_h_
#define _helperMethods_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  helperMethods.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Thomas Uchida, Chris Dembia, Shrinidhi K. Lakshmikanth,         *
 *            Ajay Seth                                                       *
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

/* Helper methods to take care of some mundane tasks. You don't need to add
anything in this file, but you should know what each of these methods does. */

#include <OpenSim/OpenSim.h>

namespace OpenSim {


//------------------------------------------------------------------------------
// Display the class name and full path name for each of the given Component's
// descendants (children, grandchildren, etc.).
//
// Examples:
//   showSubcomponentInfo(myComponent);         //show all descendant Components
//   showSubcomponentInfo<Joint>(myComponent);  //show Joint descendants only
//------------------------------------------------------------------------------
template <class C = Component>
inline void showSubcomponentInfo(const Component& comp);


//------------------------------------------------------------------------------
// Display the name of each output generated by a Component; include outputs
// generated by all the Component's descendants (children, grandchildren, etc.)
// by default (set includeDescendants=false to suppress).
//------------------------------------------------------------------------------
inline void showAllOutputs(const Component& comp, bool includeDescendants=true);


//------------------------------------------------------------------------------
// Simulate a model from an initial state. The user is repeatedly prompted to
// either begin simulating or quit. The provided state is updated (returns the
// state at the end of the simulation). Set saveStatesFile=true to save the
// states to a storage file.
//------------------------------------------------------------------------------
inline void simulate(Model& model, SimTK::State& state,
                     bool saveStatesFile=false);


//------------------------------------------------------------------------------
// Configure a PathActuator so that it wraps over a WrapObject attached to the
// specified Body. The footwork in this method is necessary because of a bug in
// GeometryPath.
//------------------------------------------------------------------------------
inline void addPathWrapHelper(ModelComponent& model,
    const std::string& pathActuatorName, const std::string& wrapObjectName,
    const std::string& bodyName);


//------------------------------------------------------------------------------
// Build a testbed for testing the device before attaching it to the hopper. We
// will attach one end of the device to ground ("/testbed/ground") and the other
// end to a sprung load ("/testbed/load").
//------------------------------------------------------------------------------
inline Model buildTestbed();


//------------------------------------------------------------------------------
// SignalGenerator is a type of Component with no inputs and only one output.
// This Component evaluates an OpenSim::Function (stored in its "function"
// property) as a function of time. We can use a SignalGenerator to design
// time-varying control inputs for testing the device.
//------------------------------------------------------------------------------
class SignalGenerator : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(SignalGenerator, Component);

public:
    OpenSim_DECLARE_PROPERTY(function, Function,
        "Function used to generate the signal (a function of time)");
    OpenSim_DECLARE_OUTPUT(signal, double, getSignal, SimTK::Stage::Time);

    SignalGenerator() { constructProperties(); }

    double getSignal(const SimTK::State& s) const {
        return get_function().calcValue(SimTK::Vector(1, s.getTime())); }

private:
    void constructProperties() { constructProperty_function(Constant(0.)); }

}; // end of SignalGenerator


//==============================================================================
//                               IMPLEMENTATIONS
//==============================================================================
template <class C>
inline void showSubcomponentInfo(const Component& comp)
{
    using std::cout; using std::endl;

    std::string className = typeid(C).name();
    const std::size_t colonPos = className.rfind(":");
    if (colonPos != std::string::npos)
        className = className.substr(colonPos+1, className.length()-colonPos);

    cout << "Class name and full path name for descendants of '"
         << comp.getName() << "' of type " << className << ":\n" << endl;

    ComponentList<C> compList = comp.getComponentList<C>();

    // Step through compList once to find the longest concrete class name.
    unsigned maxlen = 0;
    for (const C& thisComp : compList) {
        auto len = thisComp.getConcreteClassName().length();
        maxlen = std::max(maxlen, static_cast<unsigned>(len));
    }
    maxlen += 4; //padding

    // Step through compList again to print.
    for (const C& thisComp : compList) {
        const std::string thisClass = thisComp.getConcreteClassName();
        for (unsigned i=0u; i < maxlen-thisClass.length(); ++i) { cout << " "; }
        cout << "[" << thisClass << "]  " << thisComp.getFullPathName() << endl;
    }
    cout << endl;
}

inline void showAllOutputs(const Component& comp, bool includeDescendants)
{
    using std::cout; using std::endl;

    // Do not display header for Components with no outputs.
    if (comp.getNumOutputs() > 0) {
        const std::string msg = "Outputs from " + comp.getFullPathName();
        cout << msg << endl;
        for (unsigned i=0u; i<msg.size(); ++i) { cout << "="; }
        cout << endl;

        std::vector<std::string> outputNames = comp.getOutputNames();
        for (auto thisName : outputNames) { cout << "  " << thisName << endl; }
        cout << endl;
    }

    if (includeDescendants) {
        ComponentList<Component> compList = comp.getComponentList<Component>();
        for (const Component& thisComp : compList) {
            // ComponentList includes all descendants (children, grandchildren,
            // etc.) so don't call recursively here.
            showAllOutputs(thisComp, false);
        }
    }
}

inline void simulate(Model& model, SimTK::State& state, bool saveStatesFile)
{
    SimTK::State initState = state;

    // Configure the visualizer.
    if (model.getUseVisualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundType(viz.GroundAndSky).setShowSimTime(true);
        viz.drawFrameNow(state);
    }

    // Simulate until the user enters 'q'.
    while (true) {
        std::cout << "Press <Enter> to begin simulating, or 'q' followed by "
                  << "<Enter> to quit . . . " << std::endl;
        if (std::cin.get() == 'q') { break; }

        // Set up manager and simulate.
        state = initState;
        SimTK::RungeKuttaMersonIntegrator integrator(model.getSystem());
        Manager manager(model, integrator);
        manager.setInitialTime(0.); manager.setFinalTime(5.);
        manager.integrate(state);

        // Save the states to a storage file (if requested).
        if (saveStatesFile) {
            manager.getStateStorage().print("hopperStates.sto");
        }
    }
}

inline void addPathWrapHelper(ModelComponent& model,
    const std::string& pathActuatorName, const std::string& wrapObjectName,
    const std::string& bodyName)
{
    // Ensure the specified body exists.
    // Change to the following upon merge with master:
    //if (!model.hasComponent<Body>(bodyName)) { return; }
    if (model.findComponent<Body>(bodyName) == nullptr) { return; }

    auto& pathActuator = model.updComponent<PathActuator>(pathActuatorName);
    auto& body         = model.updComponent<Body>(bodyName);
    auto& wrapObject   = body.upd_WrapObjectSet().get(wrapObjectName);
    pathActuator.connect(model);
    body.connect(model);
    pathActuator.updGeometryPath().addPathWrap(wrapObject);
}

inline Model buildTestbed()
{
    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create a new OpenSim model.
    auto testbed = Model();
    testbed.setName("testbed");
    testbed.setUseVisualizer(true);
    testbed.setGravity(Vec3(0));

    // Create a 2500 kg load and add geometry for visualization.
    auto load = new Body("load", 2500., Vec3(0), Inertia(1.));
    Sphere sphere;
    sphere.setFrameName("load");
    sphere.set_radius(0.02);
    sphere.setOpacity(0.5);
    sphere.setColor(SimTK::Blue);
    load->addGeometry(sphere);
    testbed.addBody(load);

    // Attach the load to ground with a FreeJoint and set the location of the
    // load to (1,0,0).
    auto gndToLoad = new FreeJoint("gndToLoad", testbed.getGround(), *load);
    gndToLoad->getCoordinateSet()[3].setDefaultValue(1.0);
    testbed.addJoint(gndToLoad);

    // Add a spring between the ground's origin and the load.
    auto spring = new PointToPointSpring(
        testbed.getGround(), Vec3(0),   //frame G and location in G of point 1
        *load, Vec3(0),                 //frame F and location in F of point 2
        5000., 1.);                     //stiffness and rest length
    testbed.addForce(spring);

    return testbed;
}

} // end of namespace OpenSim

#endif // _helperMethods_h_
