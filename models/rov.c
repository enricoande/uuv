// 6 D.o.F. dynamics of a ROV
// excluding thrust, tether & environmental effects
// Modelled effects: hydrostatic, damping, Coriolis, added mass
//
// Enrico Anderlini, University College London, e.anderlini@ucl.ac.uk
//
// Adapted from code by
// Gordon Parker
// Michigan Technological University
// Mechanical Engineering - Engineering Mechanics Deptartment
// Houghton, MI
//
// Created : 13 September 2017
//
// Version : 1.0

#define S_FUNCTION_NAME rov
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "math.h"

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) &&\
        mxIsDouble(pVal))

#define MYPI 3.14159265358979

#define YES 1
#define NO  0

//
// Manage the size of all the inputs, outputs, states and work vectors
// using the #define statements below. This off-loads the need for
// maintaining sizes in multiple spots below.
//
// Parameters
#define P_W   0  // weight (N)
#define P_B   1  // buoyancy (N)
#define P_CG  2  // centre of gravity (m)
#define P_CB  3  // centre of buoyancy (m)
#define P_N   4  // number of elements in input
   
// continuous state indices
#define C_X   0  // (m)
#define C_Y   1  // (m)
#define C_Z   2  // (m)
#define C_PH  3  // (rad)
#define C_TH  4  // (rad)
#define C_PS  5  // (rad)
#define C_U   6  // (m/s)
#define C_V   7  // (m/s)
#define C_W   8  // (m/s)
#define C_P   9  // (rad/s)        
#define C_Q   10 // (rad/s)
#define C_R   11 // (rad/s)
#define C_N   12 // no. continuous states

// real work vector indices
#define RW_W   0 //
#define RW_B   1 //
#define RW_XG  2 //
#define RW_YG  3 //
#define RW_ZG  4 //
#define RW_XB  5 //
#define RW_YB  6 //
#define RW_ZB  7 //
#define RW_N   8 // size of real work vector

// integer work vector indices
#define IW_N   0 // size of integer work vector

// input indices
// #define I_SD    0      // state vector derivative input port #
// #define   I_SDSIZE C_N // size of input port
#define I_N    0       // # of input ports

// output indices
#define O_ST   0         // state output port #
#define   O_STSIZE C_N   // states (m and m/s)
#define O_N    1         // # of output ports

// ************************************************************************
// mdlCheckParameters: Method for checking parameter data types and sizes.
// Not required, but highly recommended to use it.
// ************************************************************************
#define MDL_CHECKPARAMETERS
#if defined(MDL_CHECKPARAMETERS)
static void mdlCheckParameters(SimStruct *S)
{
// Check 1st parameter: P_W
  {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_W)) != 1 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_W)) ) {
      ssSetErrorStatus(S,"1st parameter, P_W, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 2nd parameter: P_B
  {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_B)) != 1 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_B)) ) {
      ssSetErrorStatus(S,"2nd parameter, P_B, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 3rd parameter: P_CG
  {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_CG)) != 3 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_CG)) ) {
      ssSetErrorStatus(S,"3rd parameter, P_CG, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 4th parameter: P_CB
  {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_CB)) != 3 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_CB)) ) {
      ssSetErrorStatus(S,"4th parameter, P_CB, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
}
#endif
 
// ************************************************************************
// mdlInitializeSize: Setup i/o and state sizes
// ************************************************************************
static void mdlInitializeSizes(SimStruct *S)
{
  //-----------------------------------------------------------------------      
  //           *** P A R A M E T E R    S E T U P ***
  //-----------------------------------------------------------------------  
  //   #  Description                                Units      Dim
  //-----------------------------------------------------------------------      
  //   0. Weight                                       N         1
  //   1. Buoyancy                                     N         1
  //   2. Centre of gravity                            m         3
  //   3. Centre of buoyancy                           m         3

  ssSetNumSFcnParams(S, P_N); // total number of parameters
  
  // Catch error made by user in giving parameter list to Simulink block
  // To make the mdlCheckParameters method active, it must be called as
  // shown below. This feature is not allowable in real-time, coder use,
  // so it is conditional on the MATLAB_MEX_FILE attribute. 
  #if defined(MATLAB_MEX_FILE)
  if( ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S) )
  {
      mdlCheckParameters(S);
      if(ssGetErrorStatus(S) != NULL) return;
  }
  else return; // parameter mismatch error
  #endif

  //-----------------------------------------------------------------------        
  //         *** C O N T I N U O U S    S T A T E    S E T U P ***
  //-----------------------------------------------------------------------    
  //   #                Description                 Units       Dim
  //-----------------------------------------------------------------------  
  //   1.               state vector derivative      mix         12
  
  ssSetNumContStates(S,C_N);  // total number of continuous states           

  ssSetNumDiscStates(S,0); // total number of discrete states

  // set number of input ports
  if (!ssSetNumInputPorts(S,I_N)) return;  
  
//   // set input port widths
//   ssSetInputPortWidth(S, I_SD, I_SDSIZE);

//   // If you add new inputs, you must add an element to the list below to
//   // indicate if the input is used directly to compute an output.  
//   ssSetInputPortDirectFeedThrough(S, I_SD, NO);
  
  // specify number of output ports
  if (!ssSetNumOutputPorts(S,O_N)) return; 
  
  // specify output port widths
  ssSetOutputPortWidth(S, O_ST , O_STSIZE) ; 
  
  // setup work vectors
  // If you need several arrays or 2D arrays of work vectors, then use
  // DWork.
  ssSetNumRWork(S, RW_N); 
  ssSetNumIWork(S, IW_N);
  ssSetNumPWork(S, 0);
  ssSetNumModes(S, 0);
  ssSetNumDWork(S, 0);    
  
  // setup sample times
  ssSetNumSampleTimes(  S, 1);
  ssSetNumNonsampledZCs(S, 0);

  ssSetOptions(S, SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE);
}

// ************************************************************************
// mdlInitializeSampleTimes: Set sample times for this s-fn. Modify this
// if you want to have the S-Fn called at interesting times. Lots of 
// documentation at MathWorks regarding how to manage this. 
// ************************************************************************
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
}

// ************************************************************************
// mdlInitializeConditions: Assign state ics, and other one-off actions.
// ************************************************************************
#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS)
static void mdlInitializeConditions(SimStruct *S)
{
  // set a pointer to the continuous state vector
  real_T *x  = ssGetContStates(S);
  
  // set a pointer to the real work vector
  real_T *rw = ssGetRWork(S);
  
  // counter
  int i;
  
  //snatch and map all the needed parameters  
  const real_T *w = mxGetPr(ssGetSFcnParam(S,P_W));  
  const real_T *b = mxGetPr(ssGetSFcnParam(S,P_B));
  const real_T *cg = mxGetPr(ssGetSFcnParam(S,P_CG));  
  const real_T *cb = mxGetPr(ssGetSFcnParam(S,P_CB));
  
  // create the elements of A & B that you need, avoid all the 0 elements:
  rw[RW_W]  = w[0];
  rw[RW_B]  = b[0];
  rw[RW_XG] = cg[0];
  rw[RW_YG] = cg[1];
  rw[RW_ZG] = cg[2];
  rw[RW_XB] = cb[0];
  rw[RW_YB] = cb[1];
  rw[RW_ZB] = cb[2];
  
  // debugging
  for(i=0;i<RW_N;i++) printf("%f\n", rw[i]);
  
  // initialize the state vector
  for(i=0;i<C_N;i++) x[i]=0.0;
  
  ssSetSimStateCompliance(S,USE_DEFAULT_SIM_STATE);
}
#endif

// ************************************************************************
// mdlOutputs: Calc outputs at the start of each major integration step
// ************************************************************************
static void mdlOutputs(SimStruct *S, int_T tid)
{
  // set a pointers to the outputs
  real_T *ySt   = ssGetOutputPortSignal(S,O_ST );  // states
  
  // set a pointer to the continous state vector
  real_T *x  = ssGetContStates(S);
    
  // toss continuous states to the output port
  int_T i;   // counter
  for(i=0;i<C_N;i++) ySt[i] = x[i];
}

// ************************************************************************
// mdlUpdate: Update the discrete states
// ************************************************************************
#undef MDL_UPDATE 
#if defined(MDL_UPDATE)
static void mdlUpdate(SimStruct *S, int_T tid){}
#endif

// ************************************************************************
// mdlDerivatives: Calc state derivatives for integration
// ************************************************************************
#define MDL_DERIVATIVES 
#if defined(MDL_DERIVATIVES)
static void mdlDerivatives(SimStruct *S)
{
  real_T *x  = ssGetContStates(S); // ptr to continous states
  real_T *dx = ssGetdX(S);         // ptr to right side of x' = f(x,u,t)
  
  int_T i; // counters
  
  // Faster, but problem-specific solution:
  // set a pointer to the real work vector
  //real_T *rw = ssGetRWork(S);
  
  for (i=0;i<C_N;i++) dx[i] = x[i]+0.1;
}
#endif

// ************************************************************************
// mdlTerminate: Clean up anything that needs it
// ************************************************************************
static void mdlTerminate(SimStruct *S) { }

// Here's some stuff that is all S-Functions at the end.
#ifdef  MATLAB_MEX_FILE    // Is this file being compiled as a MEX-file?
#include "simulink.c"      // MEX-file interface mechanism 
#else
#include "cg_sfun.h"       // Code generation registration function
#endif