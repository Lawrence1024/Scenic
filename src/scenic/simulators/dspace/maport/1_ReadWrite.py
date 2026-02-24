"""
File:        1_ReadWrite.py

Description: This sample demonstrates how variables on a dSPACE platform
             can be read and written from Python .NET using the dSPACE XIL API server.

             This program uses the turn lamp simulation application from your demo directory
             MAPort\Common\SimulationApplications\<platform>.

             Adapt lines 54-64 of this file according to your dSPACE platform.

             Make sure that the dSPACE platform that is used for this demo
             is registered with ControlDesk, AutomationDesk or the Platform Management API.

             Also note in the call to the method Configure of the MAPort, the second
             parameter is set to 'false'. This means that the specified simulation application
             will not be downloaded unless there is no application loaded on the platform.
             If the specified application is already running, no further action will be taken.
             If any other application is running on the platform, an exception will be thrown.

Tip/Remarks: Objects of some XIL API types (e.g., MAPort, Capture) must be disposed at the end
             of the function. We strongly recommend to use exception handling for this purpose
             to make sure that Dispose is called even in the case of an error.

Version:     23.1

Date:        May 2023

             dSPACE GmbH shall not be liable for errors contained herein or
             direct, indirect, special, incidental, or consequential damages
             in connection with the furnishing, performance, or use of this
             file.
             Brand names or product names are trademarks or registered
             trademarks of their respective companies or organizations.

Copyright 2023, dSPACE GmbH. All rights reserved.
"""

import clr
import sys, os

# Load ASAM assemblies from the global assembly cache (GAC)
clr.AddReference("ASAM.XIL.Implementation.TestbenchFactory, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")
clr.AddReference("ASAM.XIL.Interfaces, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")

# Import XIL API .NET classes from the .NET assemblies
from ASAM.XIL.Implementation.TestbenchFactory.Testbench import TestbenchFactory
from ASAM.XIL.Interfaces.Testbench.Common.Error import TestbenchPortException
from ASAM.XIL.Interfaces.Testbench.MAPort.Enum import MAPortState
from ASAM.XIL.Interfaces.Testbench.Common.VariableRef.Enum import ValueRepresentation
from DemoHelpers import *

# The following lines must be adapted to the dSPACE platform used
#------------------------------------------------------------------------------------------------
# Set IsMPApplication to true if you are using a multiprocessor platform
IsMPSystem = False
# Use an MAPort configuration file that is suitable for your platform and simulation application
# See the folder Common\PortConfigurations for some predefined configuration files
MAPortConfigFile = r".\MAPortConfigVEOS.xml"
#--------------------------------------------------------------------------

#--------------------------------------------------------------------------
# Set the working directory for this demo script
#--------------------------------------------------------------------------
WorkingDir = os.path.dirname(sys.argv[0])
if not os.path.isdir(WorkingDir):
    WorkingDir = os.getcwd()
if not os.path.isdir(WorkingDir):
    os.mkdir(WorkingDir)

MAPortConfigFile = os.path.join(WorkingDir,MAPortConfigFile)

if __name__ == "__main__":

    DemoMAPort = None

    try:
        #--------------------------------------------------------------------------
        # Create a TestbenchFactory object; the TestbenchFactory is needed to
        # create the vendor-specific Testbench
        #--------------------------------------------------------------------------
        MyTestbenchFactory = TestbenchFactory()

        #--------------------------------------------------------------------------
        # Create a dSPACE Testbench object; the Testbench object is the central object to access
        # factory objects for the creation of all kinds of Testbench-specific objects
        #--------------------------------------------------------------------------
        MyTestbench = MyTestbenchFactory.CreateVendorSpecificTestbench("dSPACE GmbH", "XIL API", "2023-A")

        #--------------------------------------------------------------------------
        # We need an MAPortFactory to create an MAPort and also a ValueFactory to create ValueContainer
        # objects
        #--------------------------------------------------------------------------
        MyMAPortFactory = MyTestbench.MAPortFactory
        MyValueFactory = MyTestbench.ValueFactory
        MyVariableRefFactory = MyTestbench.VariableRefFactory

        #--------------------------------------------------------------------------
        # Create and configure an MAPort object and start the simulation
        #--------------------------------------------------------------------------
        print("Creating MAPort instance...")
        # Create an MAPort object using the MAPortFactory
        DemoMAPort = MyMAPortFactory.CreateMAPort("DemoMAPort")
        print("...done.\n")
        # Load the MAPort configuration
        print("Configuring MAPort...")
        DemoMAPortConfig = DemoMAPort.LoadConfiguration(MAPortConfigFile)
        # Apply the MAPort configuration
        DemoMAPort.Configure(DemoMAPortConfig, False)
        print("...done.\n")
        if DemoMAPort.State != MAPortState.eSIMULATION_RUNNING:
            # Start the simulation
            print("Starting simulation...")
            DemoMAPort.StartSimulation()
            print("...done.\n")

        #----------------------------------------------------------------------
        # Set the variables to be used in this demo
        #----------------------------------------------------------------------
        
        KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
        KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
        KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
        KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
        EGO_X = "Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1"
        
        #--------------------------------------------------------------------------
        # Read and Write the value of the TurnSignalLever variable
        #--------------------------------------------------------------------------
        ThrottleVariableRef = MyVariableRefFactory.CreateGenericVariableRef(KEY_THROTTLE, ValueRepresentation.ePhysicalValue)
        DemoMAPort.Write2(ThrottleVariableRef, MyValueFactory.CreateFloatValue(0.0))

        EGO_X_Variable_Ref = MyVariableRefFactory.CreateGenericVariableRef(EGO_X, ValueRepresentation.ePhysicalValue)
        NewReadVal = DemoMAPort.Read2(EGO_X_Variable_Ref)
        NewReadVal = convertIBaseValue(NewReadVal)
        print("Value of %s now is: %s\n" % ("EGO_X", NewReadVal.Value))
        print("")
        print("Demo successfully finished!\n")

    except TestbenchPortException as ex:
        #-----------------------------------------------------------------------
        # Display the vendor code description to get the cause of an error
        #-----------------------------------------------------------------------
        print("A TestbenchPortException occurred:")
        print("CodeDescription: %s" % ex.CodeDescription)
        print("VendorCodeDescription: %s" % ex.VendorCodeDescription)
        raise
    finally:
        #-----------------------------------------------------------------------
        # Attention: make sure to dispose the MAPort object in any case to free
        # system resources like allocated memory and also resources and services on the platform
        #-----------------------------------------------------------------------
        if DemoMAPort != None:
            DemoMAPort.Dispose()
            DemoMAPort = None