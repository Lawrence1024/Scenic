"""
File:        18_ReadWriteArrays.py

Description: This sample demonstrates how variables on a dSPACE platform
             can be read and written from Python .NET using the dSPACE XIL API server.

             This program uses the turn lamp simulation application from your demo directory
             MAPort\Common\SimulationApplications\<platform>.

             Adapt lines 60-67 of this file according to your dSPACE platform.

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

clr.AddReference("System.Collections")
import System
from System import Array

# Load ASAM assemblies from the global assembly cache (GAC)
clr.AddReference("ASAM.XIL.Implementation.TestbenchFactory, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")
clr.AddReference("ASAM.XIL.Interfaces, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")

# Import XIL API .NET classes from the .NET assemblies
from ASAM.XIL.Implementation.TestbenchFactory.Testbench import TestbenchFactory
from ASAM.XIL.Interfaces.Testbench.Common.Error import TestbenchPortException
from ASAM.XIL.Interfaces.Testbench.MAPort.Enum import MAPortState
from ASAM.XIL.Interfaces.Testbench.Common.VariableRef import IVariableRef
from ASAM.XIL.Interfaces.Testbench.Common.VariableRef.Enum import ValueRepresentation

# Import DemoHelpers for Python 3.9
from DemoHelpers import *

# The following lines must be adapted to the dSPACE platform used
#------------------------------------------------------------------------------------------------
# Set IsMPApplication to true if you are using a multiprocessor platform
IsMPSystem = False
# Use an MAPort configuration file that is suitable for your platform and simulation application
# See the folder Common\PortConfigurations for some predefined configuration files
MAPortConfigFile = r"..\Common\PortConfigurations\MAPortConfigVEOS.xml"
#--------------------------------------------------------------------------


#----------------------------------------------------------------------
# For multiprocessor platforms different variable names have to be used.
# Some variables are part of the subappliaction "masterAppl", some belong to the
# subapplication "slaveAppl"
#----------------------------------------------------------------------
if IsMPSystem:
    masterVariablesPrefix = "Platform()://masterappl/Model Root/master/CentralLightEcu/"
    slaveVariablesPrefix = "Platform()://slaveappl/Model Root/slave/FrontRearLightEcu/"

else:
    masterVariablesPrefix = "Platform()://Model Root/"
    slaveVariablesPrefix = "Platform()://Model Root/"

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
        MyVariableRef = MyTestbench.VariableRefFactory

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
        TurnSignalLever = masterVariablesPrefix + "TurnSignalLever[-1..1]/Value"
        TurnSignalLeverValueRaw = MyVariableRef.CreateGenericVariableRef(TurnSignalLever, ValueRepresentation.eRawValue)
        TurnSignalLeverValuePhysical = MyVariableRef.CreateGenericVariableRef(TurnSignalLever, ValueRepresentation.ePhysicalValue)

        VectorVariable = slaveVariablesPrefix + "FrontLightEcu/Both/Value"
        InvalidVariableName = masterVariablesPrefix + "TurnSignalLever[-1..1]/NonExistingValue"
        InvalidVariableRef_invalidName = MyTestbench.VariableRefFactory.CreateGenericVariableRef(InvalidVariableName, ValueRepresentation.eRawValue)
        InvalidVariableRef_invalidType = MyTestbench.VariableRefFactory.CreateVectorElementRef(TurnSignalLever, 0, ValueRepresentation.eRawValue)
        InvalidVariableRef_invalidIndex = MyTestbench.VariableRefFactory.CreateVectorElementRef(VectorVariable, -1, ValueRepresentation.eRawValue)
        InvalidVariableRef_invalidIndex2 = MyTestbench.VariableRefFactory.CreateVectorElementRef(VectorVariable, 5, ValueRepresentation.eRawValue)

        VectorVariableCompleteRaw = MyTestbench.VariableRefFactory.CreateGenericVariableRef(VectorVariable, ValueRepresentation.eRawValue)
        VectorVariableElementRaw = MyTestbench.VariableRefFactory.CreateVectorElementRef(VectorVariable, 0, ValueRepresentation.eRawValue)
        
        VectorVariableCompletePhysical = MyTestbench.VariableRefFactory.CreateGenericVariableRef(VectorVariable, ValueRepresentation.ePhysicalValue)
        VectorVariableElementPhysical = MyTestbench.VariableRefFactory.CreateVectorElementRef(VectorVariable, 0, ValueRepresentation.ePhysicalValue)
        
        MatrixVariable = masterVariablesPrefix + "CentralLightEcu/S-R1/S-R\nFlip-Flop1/Logic/TruthTable"
        MatrixVariableCompleteRaw = MyTestbench.VariableRefFactory.CreateGenericVariableRef(MatrixVariable, ValueRepresentation.eRawValue)
        MatrixVariableElementRaw = MyTestbench.VariableRefFactory.CreateMatrixElementRef(MatrixVariable, 5, 1, ValueRepresentation.eRawValue)
        
        MatrixVariableCompletePhysical = MyTestbench.VariableRefFactory.CreateGenericVariableRef(MatrixVariable, ValueRepresentation.ePhysicalValue)
        MatrixVariableElementPhysical = MyTestbench.VariableRefFactory.CreateMatrixElementRef(MatrixVariable, 5, 1, ValueRepresentation.ePhysicalValue)

        ValidVariableRefs = Array[IVariableRef]( [TurnSignalLeverValueRaw, TurnSignalLeverValuePhysical, VectorVariableCompleteRaw, VectorVariableElementRaw] )

        InvalidVariableRefs = Array[IVariableRef]( [InvalidVariableRef_invalidName, InvalidVariableRef_invalidType, InvalidVariableRef_invalidIndex, InvalidVariableRef_invalidIndex2] )

        ValidVariableRefsAndInvalidVariableRefs = Array[IVariableRef]( [TurnSignalLeverValueRaw, TurnSignalLeverValuePhysical, VectorVariableCompleteRaw, VectorVariableElementRaw, MatrixVariableCompletePhysical, MatrixVariableElementPhysical, InvalidVariableRef_invalidName, InvalidVariableRef_invalidType, InvalidVariableRef_invalidIndex, InvalidVariableRef_invalidIndex2] )
        
        #--------------------------------------------------------------------------
        # Read and Write the value of the TurnSignalLever variable
        #--------------------------------------------------------------------------
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(TurnSignalLeverValueRaw))
        print("Value of %s (Raw) is: %s\n" % (TurnSignalLever, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(TurnSignalLeverValuePhysical))
        print("Value of %s (Physical) is: %s\n" % (TurnSignalLever, ReadValPhysical.Value))

        DemoMAPort.Write2(TurnSignalLeverValuePhysical, MyValueFactory.CreateFloatValue(1.0))
        print("Writing value 1.0 to %s\n" % (TurnSignalLever))

        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(TurnSignalLeverValueRaw))
        print("Value of %s (Raw) now is: %s\n" % (TurnSignalLever, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(TurnSignalLeverValuePhysical))
        print("Value of %s (Physical) now is: %s\n" % (TurnSignalLever, ReadValPhysical.Value))

        DemoMAPort.Write2(TurnSignalLeverValuePhysical, MyValueFactory.CreateFloatValue(0.0))
        print("Writing value 0.0 to %s\n" % (TurnSignalLever))
        
        #--------------------------------------------------------------------------
        # Read and Write the value of the Vector variable
        #--------------------------------------------------------------------------
        # Read and write the complete vector
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(VectorVariableCompleteRaw))
        print("Value of %s (Raw) is: %s\n" % (VectorVariableCompleteRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(VectorVariableCompletePhysical))
        print("Value of %s (Physical) is: %s\n" % (VectorVariableCompletePhysical.VariableRefLabel, ReadValPhysical.Value))

        newVector = []
        originalVector = ReadValRaw.Value
        for value in ReadValRaw.Value:
            newVector.append(value + 1)
        # Modify value and read anew.
        DemoMAPort.Write2(VectorVariableCompleteRaw, MyValueFactory.CreateFloatVectorValue(Array[System.Double](newVector)))
        
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(VectorVariableCompleteRaw))
        print("Value of %s (Raw) is: %s\n" % (VectorVariableCompleteRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(VectorVariableCompletePhysical))
        print("Value of %s (Physical) is: %s\n" % (VectorVariableCompletePhysical.VariableRefLabel, ReadValPhysical.Value))
        
        # Write original value.
        DemoMAPort.Write2(VectorVariableCompleteRaw, MyValueFactory.CreateFloatVectorValue(Array[System.Double](originalVector)))

        # Read and write a single element of the vector
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(VectorVariableElementRaw))
        print("Value of %s (Raw) is: %s\n" % (VectorVariableElementRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(VectorVariableElementPhysical))
        print("Value of %s (Physical) is: %s\n" % (VectorVariableElementPhysical.VariableRefLabel, ReadValPhysical.Value))
        
        # Modify value and read anew.
        DemoMAPort.Write2(VectorVariableElementRaw, MyValueFactory.CreateFloatValue(ReadValRaw.Value + 1))
        
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(VectorVariableElementRaw))
        print("Value of %s (Raw) is: %s\n" % (VectorVariableElementRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(VectorVariableElementPhysical))
        print("Value of %s (Physical) is: %s\n" % (VectorVariableElementPhysical.VariableRefLabel, ReadValPhysical.Value))
        
        # Write original value.
        DemoMAPort.Write2(VectorVariableElementRaw, MyValueFactory.CreateFloatValue(ReadValRaw.Value - 1))
        #--------------------------------------------------------------------------
        # Read and Write the value of the Matrix variable
        #--------------------------------------------------------------------------
        # Read and write the complete matrix
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(MatrixVariableCompleteRaw))
        print("Value of %s (Raw) is: %s\n" % (MatrixVariableCompleteRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(MatrixVariableCompletePhysical))
        print("Value of %s (Physical) is: %s\n" % (MatrixVariableCompletePhysical.VariableRefLabel, ReadValPhysical.Value))
        
        newMatrix = []
        originalMatrix = ReadValRaw.Value
        for row in ReadValRaw.Value:
            newRow = []
            for value in row:
                newRow.append(not value)
            newMatrix.append(newRow)
        # Modify value and read anew.
        DemoMAPort.Write2(MatrixVariableCompleteRaw, MyValueFactory.CreateBooleanMatrixValue(Array[Array[bool]](newMatrix)))

        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(MatrixVariableCompleteRaw))
        print("Value of %s (Raw) is: %s\n" % (MatrixVariableCompleteRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(MatrixVariableCompletePhysical))
        print("Value of %s (Physical) is: %s\n" % (MatrixVariableCompletePhysical.VariableRefLabel, ReadValPhysical.Value))
        
        # Write original value.
        DemoMAPort.Write2(MatrixVariableCompleteRaw, MyValueFactory.CreateBooleanMatrixValue(Array[Array[bool]](originalMatrix)))

        # Read and write a single element of the matrix
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(MatrixVariableElementRaw))
        print("Value of %s (Raw) is: %s\n" % (MatrixVariableElementRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(MatrixVariableElementPhysical))
        print("Value of %s (Physical) is: %s\n" % (MatrixVariableElementPhysical.VariableRefLabel, ReadValPhysical.Value))
        
        # Modify value and read anew.
        DemoMAPort.Write2(MatrixVariableElementRaw, MyValueFactory.CreateBooleanValue(not ReadValRaw.Value))
                
        ReadValRaw = convertIBaseValue(DemoMAPort.Read2(MatrixVariableElementRaw))
        print("Value of %s (Raw) is: %s\n" % (MatrixVariableElementRaw.VariableRefLabel, ReadValRaw.Value))

        ReadValPhysical = convertIBaseValue(DemoMAPort.Read2(MatrixVariableElementPhysical))
        print("Value of %s (Physical) is: %s\n" % (MatrixVariableElementPhysical.VariableRefLabel, ReadValPhysical.Value))
        
        # Write original value.
        DemoMAPort.Write2(MatrixVariableElementRaw, MyValueFactory.CreateBooleanValue(not ReadValRaw.Value))
        #--------------------------------------------------------------------------

        EmptyList = DemoMAPort.CheckVariableRefs(ValidVariableRefs)
        if len(EmptyList) == 0:
            print("All given VariableRefs were valid, as expected.\n")
        FullList = DemoMAPort.CheckVariableRefs(InvalidVariableRefs)
        if len(FullList) == len(InvalidVariableRefs):
            print("All given VariableRefs were invalid, as expected.\n")
        HalfFullList = DemoMAPort.CheckVariableRefs(ValidVariableRefsAndInvalidVariableRefs)
        if len(HalfFullList) == len(InvalidVariableRefs):
            print("Only the invalid VariableRefs were returned, as expected.\n")


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