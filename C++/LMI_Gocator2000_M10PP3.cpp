//***************************************************************************************/
// 
// File name: LMI_Gocator2000_M10PP3.cpp  
//
// Synopsis:  This program contains an example of 3d reconstruction by interfacing with
//            a LMI Gocator (2000 series).
//            See the PrintHeader() function below for detailed description.
//
// Copyright © 1992-2024 Zebra Technologies Corp. and/or its affiliates
// All Rights Reserved

#include <mil.h>
#include <string.h> // for memset

// Once the Gocator 2000 SDK is installed and the project is configured correctly (see
// PrintHeader()), set GO2_INSTALLED to 1 to enable the Gocator-specific code.
// You need an actual Gocator connected to your computer.
#define GO2_INSTALLED  0

//****************************************************************************
// Example description.
//****************************************************************************
void PrintHeader()   
   {
   MosPrintf(MIL_TEXT("[EXAMPLE NAME]\n")
             MIL_TEXT("LMI_Gocator2000_M10PP3\n\n")

             MIL_TEXT("[SYNOPSIS]\n")
             MIL_TEXT("This program acquires a calibrated depth map using an LMI Gocator profile\n")
             MIL_TEXT("sensor with the Gocator 2000 API. It then converts the depth map to the MIL\n")
             MIL_TEXT("format and displays the result.\n\n")

             MIL_TEXT("[MODULES USED]\n")
             MIL_TEXT("Modules used: application, system, display, buffer, calibration,\n")
             MIL_TEXT("              image processing.\n\n"));
   }

//*****************************************************************************
// DirectX display
//*****************************************************************************

// DirectX display is only available under Windows.
#if M_MIL_USE_WINDOWS && !M_MIL_USE_RT
   #define USE_D3D_DISPLAY  1
#else
   #define USE_D3D_DISPLAY  0
#endif

#if USE_D3D_DISPLAY
   #include "MdispD3D.h"

   // D3D display parameters.
   const MIL_INT    D3D_DISPLAY_SIZE_X = 640;
   const MIL_INT    D3D_DISPLAY_SIZE_Y = 480;
   const MIL_DOUBLE MAX_Z_GAP_DISTANCE = 2.0; // in mm
#endif

//*****************************************************************************
// Gocator-specific header, constants and helper functions
//*****************************************************************************

#if GO2_INSTALLED
   // The project options must be modified so that this header is reachable.
   #include <Go2.h>

   // Change this to your Gocator's address, if different.
   const Go2Char* CAM_IP_ADDRESS  = reinterpret_cast<const Go2Char*>("192.168.1.10");

   // Timeouts related to acquisition time.
   const Go2UInt64 RECEIVE_TIMEOUT = 20000; // in us
   const Go2UInt64 MAX_WAIT_TIME   = 10;    // in s

   // Helper function for SAFE_GO2_CALL.
   bool Go2CallSucceeded(Go2Status CamStatus, const MIL_TEXT_CHAR* FunctionName)
      {
      if (!GO2_SUCCESS(CamStatus))
         {
         MosPrintf(MIL_TEXT("Error in '%s' (status: %d)\n"), FunctionName, static_cast<int>(CamStatus));
         MosPrintf(MIL_TEXT("\nPress <Enter> to end.\n"));
         MosGetch();
         return false;
         }
      return true;
      }
   
   //*****************************************************************************
   // Pass the status of each Go2 call to this macro. It will print an error
   // with the given function name and exit the current function if the Go2
   // function failed.
   //
   //   CamStatus       (in)  Return status of the Go2 call.
   //   FunctionName    (in)  Name of the function to print in the error message.
   //*****************************************************************************
   #define SAFE_GO2_CALL(CamStatus, FunctionName)  \
      if (!Go2CallSucceeded(CamStatus, FunctionName)) return
#endif

// Helper function for EXAMPLE_ASSERT.
bool AssertSucceeded(bool Condition, const MIL_TEXT_CHAR* Message)
   {
   if (!Condition)
      {
      MosPrintf(Message);
      MosPrintf(MIL_TEXT("\nPress <Enter> to end.\n"));
      MosGetch();
      return false;
      }
   return true;
   }
   
//*****************************************************************************
// Tests necessary conditions using this macro. It will print an error
// with the given string and exit the current function if the condition
// is false.
//
//   Condition       (in)  The condition assumed to be true.
//   Message         (in)  Error message.
//*****************************************************************************
#define EXAMPLE_ASSERT(Condition, Message)   if (!AssertSucceeded(Condition, Message)) return

//*****************************************************************************
// Structure containing objects that need deallocation (MIL and Go2, if enabled)
//*****************************************************************************
struct SObjects
   {
   MIL_ID MilApplication ;    // application identifier
   MIL_ID MilSystem      ;    // system identifier
   MIL_ID MilTemp32Buffer;    // temporary buffer used during conversion from Go2 to MIL
   MIL_ID MilDepthMap    ;    // Depth map buffer identifier
   MIL_ID MilIntensityMap;    // Intensity map buffer identifier

#if GO2_INSTALLED
   Go2System CamSystem;       // Go2 system object
   Go2Data   CamData  ;       // Go2 object containing whole part data
#endif
   };

//*****************************************************************************
// Helper function prototypes/
//*****************************************************************************

void AcquireDepthMap(SObjects* ObjectsPtr);
void DisplayDepthMap(const SObjects* ObjectsPtr);

//*****************************************************************************
// Main.
//*****************************************************************************
int MosMain()
   {
   SObjects Objects;
   memset(&Objects, 0, sizeof(Objects));

   PrintHeader();

#if !GO2_INSTALLED
   MosPrintf(MIL_TEXT("This example is designed to be used with an LMI Gocator profile sensor and\n"));
   MosPrintf(MIL_TEXT("the Gocator 2000 SDK. To run the example:\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Install the Gocator 2000 SDK.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Follow the Gocator 2000 Quick Start Guide:\n"));
   MosPrintf(MIL_TEXT("  - Connect the Gocator sensor to your computer.\n"));
   MosPrintf(MIL_TEXT("  - Change your network adapter settings.\n"));
   MosPrintf(MIL_TEXT("  - Connect to the Gocator sensor using a web browser.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Use the web browser interface to configure and calibrate the Gocator sensor:\n"));
   MosPrintf(MIL_TEXT("  - In the Setup pane, set the operation mode to Whole Part and configure the\n"));
   MosPrintf(MIL_TEXT("    settings according to your setup.\n"));
   MosPrintf(MIL_TEXT("  - In the Output pane, ensure that the Ethernet protocol is set to Gocator\n"));
   MosPrintf(MIL_TEXT("    and that the Part (and optionally, the Intensity) check boxes are checked.\n"));
   MosPrintf(MIL_TEXT("  - Refer to the Gocator User Guide for more information.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- If necessary, compile the Go2 shared library for your platform. Refer to\n"));
   MosPrintf(MIL_TEXT("  the Gocator 2000 API Manual for more information. The following instructions\n"));
   MosPrintf(MIL_TEXT("  assume a release build.\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Copy the shared library to your system path. Under Windows, copy\n"));
   MosPrintf(MIL_TEXT("    <Go2_SDK>\\bin\\<platform>\\Go2.dll\n"));
   MosPrintf(MIL_TEXT("  to\n"));
   MosPrintf(MIL_TEXT("    %%SYSTEMROOT%%\\System32\\   (usually C:\\Windows\\System32\\)\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Add the paths to the header files and library files of the Gocator SDK to\n"));
   MosPrintf(MIL_TEXT("  the example project files. If you are using Visual Studio, open the Property\n"));
   MosPrintf(MIL_TEXT("  Pages of the LMI_Gocator2000_M10PP3 project. Then, under Configuration Properties,\n"));
   MosPrintf(MIL_TEXT("  you must:\n"));
   MosPrintf(MIL_TEXT("  - Add <Go2_SDK>\\include to\n"));
   MosPrintf(MIL_TEXT("    C/C++->General->Additional Include Directories\n"));
   MosPrintf(MIL_TEXT("  - Add <Go2_SDK>\\lib\\<platform> to\n"));
   MosPrintf(MIL_TEXT("    Linker->General->Additional Library Directories\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Link with the Gocator library. If you are using Visual Studio, open the\n"));
   MosPrintf(MIL_TEXT("  Property Pages of the LMI_Gocator2000_M10PP3 project. Then, under Configuration\n"));
   MosPrintf(MIL_TEXT("  Properties, you must:\n"));
   MosPrintf(MIL_TEXT("  - Add Go2.lib to Linker->Input->Additional Dependencies\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("- Update the example code:\n"));
   MosPrintf(MIL_TEXT("  - Set the GO2_INSTALLED define to 1.\n"));
   MosPrintf(MIL_TEXT("  - Change the CAM_IP_ADDRESS variable, if necessary.\n"));
   MosPrintf(MIL_TEXT("  - Recompile the example.\n"));
   MosPrintf(MIL_TEXT("\n\n"));
   MosPrintf(MIL_TEXT("The example has been tested with the following setups:\n"));
   MosPrintf(MIL_TEXT("- Windows 7 64-bit.\n"));
   MosPrintf(MIL_TEXT("- Gocator 2030-2M-01 (versions 3.4.2.2 and 3.6.5.33).\n"));
   MosPrintf(MIL_TEXT("- Gocator 2000 API   (versions 3.4.2.2 and 3.6.5.33).\n"));
   MosPrintf(MIL_TEXT("\n"));
   MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
   MosGetch();
#else
   AcquireDepthMap(&Objects);

   // Cleanup Go2 objects.
   if (Objects.CamData != GO2_NULL)
      Go2Data_Destroy(Objects.CamData);

   if (Objects.CamSystem != GO2_NULL)
      {
      Go2System_Stop(Objects.CamSystem);     // stop Gocator system
      Go2System_Destroy(Objects.CamSystem);  // destroy system handle
      }

   Go2Api_Terminate(); // terminate Go2 API
#endif

   // Cleanup MIL objects.
   if (Objects.MilIntensityMap != M_NULL) MbufFree(Objects.MilIntensityMap);
   if (Objects.MilDepthMap     != M_NULL) MbufFree(Objects.MilDepthMap    );
   if (Objects.MilTemp32Buffer != M_NULL) MbufFree(Objects.MilTemp32Buffer);
   if (Objects.MilSystem       != M_NULL) MsysFree(Objects.MilSystem      );
   if (Objects.MilApplication  != M_NULL) MappFree(Objects.MilApplication );
   return 0;
   }

//*****************************************************************************
// Initializes the Gocator in "whole part" mode, acquires a depth map, converts
// it to the MIL format, then shows the result.
//
//   ObjectsPtr (in-out)  Structure containing all objects needing deallocation
//*****************************************************************************
void AcquireDepthMap(SObjects* ObjectsPtr)
   {
   // Information about the depth map
   bool       GotPartData   = false;
   MIL_INT    DepthMapSizeX = 0;
   MIL_INT    DepthMapSizeY = 0;
   MIL_INT16* pDepthData    = 0;
   MIL_INT    DepthMapPitch = 0;

   // Information about the intensity map
   bool       GotPartIntensityData = false;
   MIL_INT    IntensityMapSizeX    = 0;
   MIL_INT    IntensityMapSizeY    = 0;
   MIL_UINT8* pIntensityData       = 0;
   MIL_INT    IntensityMapPitch    = 0;

   // Calibration information
   MIL_DOUBLE XOffset     = 0.0;
   MIL_DOUBLE YOffset     = 0.0;
   MIL_DOUBLE ZOffset     = 0.0;
   MIL_DOUBLE XResolution = 0.0;
   MIL_DOUBLE YResolution = 0.0;
   MIL_DOUBLE ZResolution = 0.0;

   // MIL buffers that will be created on top of the camera output
   MIL_ID CamDepthMap     = M_NULL;
   MIL_ID CamIntensityMap = M_NULL;

   // Allocate and initialize MIL objects.
   MappAlloc(M_NULL, M_DEFAULT, &ObjectsPtr->MilApplication);
   MsysAlloc(ObjectsPtr->MilApplication, M_SYSTEM_HOST, M_DEFAULT, M_DEFAULT, &ObjectsPtr->MilSystem);

   MosPrintf(MIL_TEXT("The Gocator profile sensor will be initialized to acquire a depth map.\n"));
   MosPrintf(MIL_TEXT("Press <Enter> to continue.\n\n"));
   MosGetch();

#if GO2_INSTALLED
   // Allocate and initialize Go2 objects.
   Go2IPAddress Address;
   SAFE_GO2_CALL( Go2Api_Initialize()                                                , MIL_TEXT("Go2Api_Initialize")     ); // initialize Go2 API
   SAFE_GO2_CALL( Go2System_Construct(&ObjectsPtr->CamSystem)                        , MIL_TEXT("Go2System_Construct")   ); // construct Go2 system object
   SAFE_GO2_CALL( Go2IPAddress_Parse(CAM_IP_ADDRESS, &Address)                       , MIL_TEXT("Go2IPAddress_Parse")    ); // convert ip address string to Go2IPAddress object
   SAFE_GO2_CALL( Go2System_Connect(ObjectsPtr->CamSystem, Address)                  , MIL_TEXT("Go2System_Connect")     ); // connect to Gocator system at default address
   SAFE_GO2_CALL( Go2System_ConnectData(ObjectsPtr->CamSystem, GO2_NULL, GO2_NULL)   , MIL_TEXT("Go2System_ConnectData") ); // establish a Gocator data connection
   SAFE_GO2_CALL( Go2System_SetMode(ObjectsPtr->CamSystem, GO2_MODE_PART_MEASUREMENT), MIL_TEXT("Go2System_SetMode")     ); // ensure Gocator is in Whole Part Mode
   SAFE_GO2_CALL( Go2System_Start(ObjectsPtr->CamSystem)                             , MIL_TEXT("Go2System_Start")       ); // Start Gocator system

   MosPrintf(MIL_TEXT("Acquiring profiles... "));

   // Waiting for Whole Part data.
   const MIL_INT MAX_ITER = static_cast<MIL_INT>(MAX_WAIT_TIME * 1000000 / RECEIVE_TIMEOUT) + 1;
   MIL_INT Iter = 0;
   for (; Iter < MAX_ITER; ++Iter)
      {
      if (GO2_SUCCESS(Go2System_ReceiveData(ObjectsPtr->CamSystem, RECEIVE_TIMEOUT, &ObjectsPtr->CamData)))
         break;
      }
   EXAMPLE_ASSERT(Iter < MAX_ITER,
                  MIL_TEXT("didn't receive any data.\n")
                  MIL_TEXT("Maybe there is nothing in the Gocator's field of view?\n"));

   // Retrieve the whole part results.
   for (Go2UInt32 i = 0; i < Go2Data_ItemCount(ObjectsPtr->CamData); ++i)
      {
      Go2Data DataItem = Go2Data_ItemAt(ObjectsPtr->CamData, i);
      switch (Go2Object_Type(DataItem))
         {
         case GO2_TYPE_PART_DATA:
            {
            GotPartData = true;

            // Get depth map buffer attributes.
            DepthMapSizeX = static_cast<MIL_INT>( Go2PartData_Width(DataItem) );
            DepthMapSizeY = static_cast<MIL_INT>( Go2PartData_Length(DataItem) );
            pDepthData = Go2PartData_RangesAt(DataItem, 0);
            MIL_INT16* pSecondLineOfData = Go2PartData_RangesAt(DataItem, 1);
            DepthMapPitch = pSecondLineOfData - pDepthData;

            // Get calibration info.
            XOffset     = Go2PartData_XOffset    (DataItem);
            YOffset     = Go2PartData_YOffset    (DataItem);
            ZOffset     = Go2PartData_ZOffset    (DataItem);
            XResolution = Go2PartData_XResolution(DataItem);
            YResolution = Go2PartData_YResolution(DataItem);
            ZResolution = Go2PartData_ZResolution(DataItem);
            }
            break;

         case GO2_TYPE_PART_INTENSITY_DATA:
            {
            GotPartIntensityData = true;

            // Get intensity map buffer attributes.
            IntensityMapSizeX = static_cast<MIL_INT>( Go2PartIntensityData_Width(DataItem) );
            IntensityMapSizeY = static_cast<MIL_INT>( Go2PartIntensityData_Length(DataItem) );
            pIntensityData = Go2PartIntensityData_ValuesAt(DataItem, 0);
            MIL_UINT8* pSecondLineOfData = Go2PartIntensityData_ValuesAt(DataItem, 1);
            IntensityMapPitch = pSecondLineOfData - pIntensityData;
            }
            break;

         default:
            // Ignore the other results.
            break;
         }
      }
#endif

   // Sanity checks.
   EXAMPLE_ASSERT(GotPartData,
                  MIL_TEXT("data received, but without any part data.\n")
                  MIL_TEXT("No Ethernet output enabled?\n"));
   if (GotPartIntensityData)
      {
      EXAMPLE_ASSERT(DepthMapSizeX == IntensityMapSizeX && DepthMapSizeY == IntensityMapSizeY,
                     MIL_TEXT("depth map size differs from the intensity map size.\n"));
      }

   MosPrintf(MIL_TEXT("OK.\n\n"));

   // Allocate MIL buffers.
   MbufAlloc2d(ObjectsPtr->MilSystem, DepthMapSizeX, DepthMapSizeY, 32+M_SIGNED  , M_IMAGE+M_PROC       , &ObjectsPtr->MilTemp32Buffer);
   MbufAlloc2d(ObjectsPtr->MilSystem, DepthMapSizeX, DepthMapSizeY, 16+M_UNSIGNED, M_IMAGE+M_PROC+M_DISP, &ObjectsPtr->MilDepthMap    );
   if (GotPartIntensityData)
      MbufAlloc2d(ObjectsPtr->MilSystem, IntensityMapSizeX, IntensityMapSizeY, 8+M_UNSIGNED, M_IMAGE+M_PROC, &ObjectsPtr->MilIntensityMap);

   // Create MIL buffers on top of the camera data.
   MbufCreate2d(M_DEFAULT_HOST, DepthMapSizeX, DepthMapSizeY, 16+M_SIGNED, M_IMAGE+M_PROC, M_HOST_ADDRESS+M_PITCH, DepthMapPitch, pDepthData, &CamDepthMap);
   if (GotPartIntensityData)
      MbufCreate2d(M_DEFAULT_HOST, IntensityMapSizeX, IntensityMapSizeY, 8+M_UNSIGNED, M_IMAGE+M_PROC, M_HOST_ADDRESS+M_PITCH, IntensityMapPitch, pIntensityData, &CamIntensityMap);

   // Convert Gocator depth map to MIL depth map.
   // Gocator transmits range data as 16-bit signed integers, where missing data is the
   // 'most negative' value, -32768. MIL uses 16-bit unsigned integers, where missing data
   // is the 'most positive' value, 65535. Conversion steps:
   //   Add 32767 (0x7FFF):    [-32768, 32767] becomes [-1, 65534]
   //   Cast to 16+M_UNSIGNED: -1 becomes 65535 (missing data)

   MimArith(CamDepthMap, 0x7FFF, ObjectsPtr->MilTemp32Buffer, M_ADD_CONST);  // Add 0x7FFF
   MbufCopy(ObjectsPtr->MilTemp32Buffer, ObjectsPtr->MilDepthMap);           // "Cast" to 16+M_UNSIGNED

   // Gocator transmits intensity data as an 8-bit grayscale image of identical width and
   // height as the corresponding height map.
   // Copy the intensity map (no conversion needed).
   if (GotPartIntensityData)
      MbufCopy(CamIntensityMap, ObjectsPtr->MilIntensityMap);

   // Free the buffers on top of the camera data.
   if (GotPartIntensityData)
      MbufFree(CamIntensityMap);
   MbufFree(CamDepthMap);

   // To translate 16-bit range data to engineering units, the calculation for each point is: 
   //          X: XOffset + columnIndex * XResolution 
   //          Y: YOffset + rowIndex    * YResolution
   //          Z: ZOffset + height_map[rowIndex][columnIndex] * ZResolution

   // Transform the calibration. Compensate for the +0x7FFF operation.
   ZOffset -= ZResolution*0x7FFF;

   // Calibrate the depth map.
   McalUniform(ObjectsPtr->MilDepthMap, XOffset, YOffset, XResolution, YResolution, 0.0, M_DEFAULT);
   McalControl(ObjectsPtr->MilDepthMap, M_GRAY_LEVEL_SIZE_Z, ZResolution);
   McalControl(ObjectsPtr->MilDepthMap, M_WORLD_POS_Z      , ZOffset    );
      
   // Calibrate the intensity map.
   if (GotPartIntensityData)
      McalUniform(ObjectsPtr->MilIntensityMap, XOffset, YOffset, XResolution, YResolution, 0.0, M_DEFAULT);

   DisplayDepthMap(ObjectsPtr);
   }

//*****************************************************************************
// Displays the depth map using a DirectX window, if available. Otherwise,
// allocates a MIL display and displays the depth map.
//
//   ObjectsPtr (in-out)  Structure containing all objects needing deallocation
//
//*****************************************************************************
void DisplayDepthMap(const SObjects* ObjectsPtr)
   {
   MIL_ID MilDisplay = M_NULL;
#if USE_D3D_DISPLAY
   // Try to allocate D3D display.
   MIL_DISP_D3D_HANDLE DispHandle = MdepthD3DAlloc(ObjectsPtr->MilDepthMap,
                                                   ObjectsPtr->MilIntensityMap,
                                                   D3D_DISPLAY_SIZE_X,
                                                   D3D_DISPLAY_SIZE_Y,
                                                   M_DEFAULT,
                                                   M_DEFAULT,
                                                   M_DEFAULT,
                                                   M_DEFAULT,
                                                   M_DEFAULT,
                                                   MAX_Z_GAP_DISTANCE,
                                                   0);

   if (DispHandle != NULL)
      {
      MosPrintf(MIL_TEXT("The acquired depth map was converted to the MIL format and is now\n")
                MIL_TEXT("displayed in a Direct3D window.\n\n"));
      MdispD3DShow(DispHandle);
      MdispD3DPrintHelp(DispHandle);
      }
   else
#endif
      {
      // If DirectX is not available on the platform or the allocation failed, use a 2D
      // MIL display.
      MosPrintf(MIL_TEXT("The acquired depth map was converted to the MIL format and is now\n")
                MIL_TEXT("displayed in a MIL display.\n\n"));
      MdispAlloc(ObjectsPtr->MilSystem, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_WINDOWED, &MilDisplay);
      MdispControl(MilDisplay, M_VIEW_MODE, M_AUTO_SCALE);
      MdispSelect(MilDisplay, ObjectsPtr->MilDepthMap);
      }

   MosPrintf(MIL_TEXT("Press <Enter> to end.\n"));
   MosGetch();

   // Cleanup.
#if USE_D3D_DISPLAY
   if (DispHandle != M_NULL)
      {
      MdispD3DHide(DispHandle);
      MdispD3DFree(DispHandle);
      }
#endif
   if (MilDisplay != M_NULL)
      {
      MdispSelect(MilDisplay, M_NULL);
      MdispFree(MilDisplay);
      }
   }
