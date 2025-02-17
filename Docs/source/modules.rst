Functions and Simulink models
=============================

.. toctree::
   :maxdepth: 4

Functions
**********
Pre-simulation
---------------
Get_simulation_params
^^^^^^^^^^^^
.. automodule:: Src.Common.Get_simulation_params
  :members:
  :show-inheritance:
  :undoc-members:
initAllSimParams_DE2019
^^^^^^^^^^^^
.. automodule:: Src.Common.initAllSimParams_DE2019
  :members:
  :show-inheritance:
  :undoc-members:
initAllSimParams_DE2019
^^^^^^^^^^^^
.. automodule:: Src.Common.initAllSimParams_DE2019
  :members:
  :show-inheritance:
  :undoc-members:
initAllSimParams_DE2019
^^^^^^^^^^^^
.. automodule:: Src.Common.initAllSimParams_DE2019
  :members:
  :show-inheritance:
  :undoc-members:
Offline_visualisation
----------------------
enhance_plot
^^^^^^^^^^^^
.. automodule:: Src.Offline_visualisation.enhance_plot
  :members:
  :show-inheritance:
  :undoc-members:
extractSignalOfLastCycle_nD
^^^^^^^^^^^^
.. automodule:: Src.Offline_visualisation.extractSignalOfLastCycle_nD
  :members:
  :show-inheritance:
  :undoc-members:
extractSignalOfLastCycle2
^^^^^^^^^^^^
.. automodule:: Src.Offline_visualisation.extractSignalOfLastCycle2
  :members:
  :show-inheritance:
  :undoc-members:
extractSignalOfLastCycle3D
^^^^^^^^^^^^
.. automodule:: Src.Offline_visualisation.extractSignalOfLastCycle3D
  :members:
  :show-inheritance:
  :undoc-members:
Offline_visualisation_power
^^^^^^^^^^^^
.. automodule:: Src.Offline_visualisation.Offline_visualisation_power
  :members:
  :show-inheritance:
  :undoc-members:
Theoretical_Pcheck
^^^^^^^^^^^^
.. automodule:: Src.Offline_visualisation.Theoretical_Pcheck
  :members:
  :show-inheritance:
  :undoc-members:
Post-simulation & Animation
--------------------------
Replace_MdlRef_SubSys
^^^^^^^^^^^^
.. automodule:: Src.Extra.Replace_MdlRef_SubSys
  :members:
  :show-inheritance:
  :undoc-members:
slcopy_mdl2subsys
^^^^^^^^^^^^
.. automodule:: Src.Extra.slcopy_mdl2subsys
  :members:
  :show-inheritance:
  :undoc-members:
animate_flightpath
^^^^^^^^^^^^
.. automodule:: Src.Extra.animate_flightpath
  :members:
  :show-inheritance:
  :undoc-members:
animation_aircraft
^^^^^^^^^^^^
.. automodule:: Src.Extra.animation_aircraft
  :members:
  :show-inheritance:
  :undoc-members:
animation_colorbar
^^^^^^^^^^^^
.. automodule:: Src.Extra.animation_colorbar
  :members:
  :show-inheritance:
  :undoc-members:
animation_simtime
^^^^^^^^^^^^
.. automodule:: Src.Extra.animation_simtime
  :members:
  :show-inheritance:
  :undoc-members:
animation_tether
^^^^^^^^^^^^
.. automodule:: Src.Extra.animation_tether
  :members:
  :show-inheritance:
  :undoc-members:
exportToPreviousVersion
^^^^^^^^^^^^
.. automodule:: Src.Extra.exportToPreviousVersion
  :members:
  :show-inheritance:
  :undoc-members:
Update_previous_version
^^^^^^^^^^^^
.. automodule:: Src.Extra.Update_previous_version
  :members:
  :show-inheritance:
  :undoc-members:
Video_from_simOut
^^^^^^^^^^^^
.. automodule:: Src.Extra.Video_from_simOut
  :members:
  :show-inheritance:
  :undoc-members:

Flightpaths
--------------------------
.. automodule:: Src.Common.Flightpaths
  :members:
  :show-inheritance:
  :undoc-members:

Main simulink model
********************
The following image shows the root level of the simulink model. This shows how the different modules work together.
This level is similar for both 3DoF and 6DoF simulations.

.. simulink-diagram:: Dyn_PointMass_r2019b
  :dir: ../../Src/3DoF/Simulink
  :addpath: ../../Common/Environment;../../Common/FlightController;
      ../../Common/Flightpaths;../../Common/Flightpaths/boothLem;
      ../../Common/GroundStation;../../Common/Tether;../../Common/Visualisation;
      ../..;../../..

Simulink parts
***************

Environment
------------
The following image shows the environment subsystem of the simulink model.
This subsystem is similar for both 3DoF and 6DoF simulations.
It is used to determine the wind speed at each tether particle and at the kite.

.. simulink-diagram:: Dyn_PointMass_r2019b
  :dir: ../../Src/3DoF/Simulink
  :addpath: ../../Common/Environment;../../Common/FlightController;
      ../../Common/Flightpaths;../../Common/Flightpaths/boothLem;
      ../../Common/GroundStation;../../Common/Tether;../../Common/Visualisation;
      ../..;../../..
  :subsystem: Environment


.. automodule:: Src.Common.Environment
  :members:
  :show-inheritance:
  :undoc-members:

Flight controller
------------------
The following image shows the root level of the flight controller in the case of a 3DoF simulation.

.. simulink-diagram:: Dyn_PointMass_r2019b
  :dir: ../../Src/3DoF/Simulink
  :addpath: ../../Common/Environment;../../Common/FlightController;
      ../../Common/Flightpaths;../../Common/Flightpaths/boothLem;
      ../../Common/GroundStation;../../Common/Tether;../../Common/Visualisation;
      ../..;../../..
  :subsystem: AirborneSystem/FlightControlSystem

The following image shows the root level of the flight controller in the case of a 6DoF simulation.

.. simulink-diagram:: Dyn_6DoF_v2_0_r2019b
  :dir: ../../Src/6DoF/Simulink
  :addpath: ../../Common/Environment;../../Common/FlightController;
      ../../Common/Flightpaths;../../Common/Flightpaths/boothLem;
      ../../Common/GroundStation;../../Common/Tether;../../Common/Visualisation;
      ../..;../../..
  :subsystem: AirborneSystem/FlightControlSystem

.. automodule:: Src.Common.FlightController
  :members:
  :show-inheritance:
  :undoc-members:

Flightpaths
^^^^^^^^^^^^
These functions are called by the PathFollowingController.
Not all required functions are shown here. Some functions are only documented inside simulink.

.. .. simulink-diagram:: Dyn_PointMass_r2019b
..     :dir: ../../Src/3DoF/Simulink
..     :addpath: ../../Common/Environment;../../Common/FlightController;
..         ../../Common/Flightpaths;../../Common/Flightpaths/boothLem;
..         ../../Common/GroundStation;../../Common/Tether;../../Common/Visualisation;
..         ../..;../../..
..     :subsystem: AirborneSystem/FlightControlSystem/PathFollowingController

.. automodule:: Src.Common.Flightpaths
  :members:
  :show-inheritance:
  :undoc-members:

.. automodule:: Src.Common.Flightpaths.boothLem
  :members:
  :show-inheritance:
  :undoc-members:

Ground station
---------------
The following image shows the ground station subsystem of the simulink model.
This subsystem is similar for both 3DoF and 6DoF simulations.
Unfortunately, no image can be included of the state machine, however this can be viewed inside simulink.
Basically the state machine determines in what flight mode the kite needs to be and what the tether force set point is corresponding to that mode.

.. simulink-diagram:: Dyn_PointMass_r2019b
  :dir: ../../Src/3DoF/Simulink
  :addpath: ../../Common/Environment;../../Common/FlightController;
      ../../Common/Flightpaths;../../Common/Flightpaths/boothLem;
      ../../Common/GroundStation;../../Common/Tether;../../Common/Visualisation;
      ../..;../../..
  :subsystem: GroundSystem/StateMachine

.. automodule:: Src.Common.GroundStation
  :members:
  :show-inheritance:
  :undoc-members:

Tether dynamics
----------------

The functions are described inside simulink.
The following image shows the what the inputs and outputs are of the tether module.
This subsystem is similar for both 3DoF and 6DoF simulations.


.. simulink-diagram:: Dyn_PointMass_r2019b
  :dir: ../../Src/3DoF/Simulink
  :addpath: ../../Common/Environment;../../Common/FlightController;
      ../../Common/Flightpaths;../../Common/Flightpaths/boothLem;
      ../../Common/GroundStation;../../Common/Tether;../../Common/Visualisation;
      ../..;../../..
  :subsystem: ParticleTetherModel/Tether

.. automodule:: Src.Common.Tether
  :members:
  :show-inheritance:
  :undoc-members:

Offline visualisation
----------------------
These functions are used to visualise the simulation output.
They can be called by the user for each parameter being logged by simulink.
These is done outside of the simulation itself, hence the word offline.

.. automodule:: Src.Offline_visualisation
  :members:
  :show-inheritance:
  :inherited-members:
  :undoc-members:
