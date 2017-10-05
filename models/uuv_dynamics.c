// 6 D.o.F. dynamics of an UUV
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
// Version : 1.1
// Modifications:
// 05/10/2017   The block has been modified to compute the current velocity
//              in the body-fixed reference frame.

#define S_FUNCTION_NAME uuv_dynamics
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
#define P_IC  4  // initial conditions
#define P_M   5  // inverse of the mass matrix
#define P_MA  6  // added mass matrix
#define P_MB  7  // rigid body mass matrix
#define P_DL  8  // rigid body linear damping matrix
#define P_DQ  9  // rigid body quadratic damping matrix
#define P_N  10  // number of elements in input
   
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
#define RW_W   0 // weight
#define RW_B   1 // buoyancy
#define RW_XG  2 // LCG
#define RW_YG  3 // 
#define RW_ZG  4 // VCG
#define RW_XB  5 // LCB
#define RW_YB  6 //
#define RW_ZB  7 // VCB
#define RW_N   8 // size of real work vector
        
// dynamic work vector indices
#define DW_R   0 // restoring force vector
#define     DW_RSIZE  6
#define DW_D   1 // damping force vector
#define     DW_DSIZE  6
#define DW_C   2 // Coriolis force vector
#define     DW_CSIZE  6
#define DW_TT  3 // translational transformation matrix
#define     DW_TTSIZE 9 
#define DW_RT  4 // rotational transformation matrix
#define     DW_RTSIZE 9
#define DW_VR  5 // relative velocity vector
#define     DW_VRSIZE 6
#define DW_N   6 // size of dynamic work vector
        
// integer work vector indices
#define IW_N   0 // size of integer work vector

// input indices
#define I_T    0       // thurst vector
#define   I_TSIZE   6  // size of input port
#define I_VC   1       // current velocity vector
#define   I_VCSIZE  6  // size of input port
#define I_N    2       // # of input ports

// output indices
#define O_ST   0         // state output port #
#define   O_STSIZE C_N   // states (m and m/s)
#define O_F    1
#define   O_FSIZE  18    // restoring, damping & Coriolis forces (x6)
#define O_VR   2
#define   O_VRSIZE  6    // relative velocity (m/s)
#define O_N    3         // # of output ports

// ---------------------  Support Functions  ------------------------------
// ************************************************************************
// skew_symmetric: Function that returns the skew symmetric matrix of the
// given vector. Note that the vector must be of size (3,1) and the matrix
// is of size (3,3). The matrix will in fact be returned in vectorized 
// form.
// Input:
// S: skew symmetric matrix (pre-allocated memory will be filled);
// v: (3,1) vector;
// i: pointer to start of vector of interest.
// ************************************************************************
void skew_symmetric(real_T *S, const real_T *v, const int_T i)
{
    // Return the vectorized skew symmetric matrix:
    S[0] = 0.0;
    S[1] = -v[2+i];
    S[2] = v[1+i];
    S[3] = v[2+i];
    S[4] = 0.0;
    S[5] = -v[0+i];
    S[6] = -v[1+i];
    S[7] = v[0+i];
    S[8] = 0.0;
}

// ************************************************************************
// restoring_force: Function that returns the restoring force on the UUV.
// N.B.: For greater efficiency, a dynamic work vector is used.
// ************************************************************************
void restoring_force(SimStruct *S)
{
    // Set a pointer to the dynamic work vector for the restoring force:
    real_T *dw = (real_T*) ssGetDWork(S,DW_R);
    // Set a pointer to the real work vector:
    real_T *rw = ssGetRWork(S);
    // Set a pointer to the continuous state vector:
    real_T *x  = ssGetContStates(S);
    
    // Compute the restoring force: 
    dw[0] = (rw[RW_W]-rw[RW_B])*sin(x[C_TH]);
    dw[1] = (rw[RW_B]-rw[RW_W])*cos(x[C_TH])*sin(x[C_PH]);
    dw[2] = (rw[RW_B]-rw[RW_W])*cos(x[C_TH])*cos(x[C_PH]);
    dw[3] = (rw[RW_B]*rw[RW_YB]-rw[RW_W]*rw[RW_YG])*cos(x[C_TH])\
            *cos(x[C_PH])+(rw[RW_W]*rw[RW_ZG]-rw[RW_B]*rw[RW_ZB])\
            *cos(x[C_TH])*sin(x[C_PH]);
    dw[4] = (rw[RW_W]*rw[RW_ZG]-rw[RW_B]*rw[RW_ZB])*sin(x[C_TH])\
            +(rw[RW_W]*rw[RW_XG]-rw[RW_B]*rw[RW_XB])*cos(x[C_TH])\
            *cos(x[C_PH]);
    dw[5] = (rw[RW_B]*rw[RW_XB]-rw[RW_W]*rw[RW_XG])*cos(x[C_TH])\
            *sin(x[C_PH])+(rw[RW_B]*rw[RW_YB]-rw[RW_W]*rw[RW_YG])\
            *sin(x[C_TH]);
}

// ************************************************************************
// damping_force: Function that returns the damping force on the UUV.
// N.B.: For greater efficiency, a dynamic work vector is used.
// ************************************************************************
void damping_force(SimStruct *S)
{
    // Set a pointer to the dynamic work vector for the damping force:
    real_T *dw = (real_T*) ssGetDWork(S,DW_D);
    // Set a pointer to the dynamic work vector for the relative velocity:
    real_T *vr = (real_T*) ssGetDWork(S,DW_VR);
    // Snatch and map the linear damping matrix:
    const real_T *DL1D = mxGetPr(ssGetSFcnParam(S,P_DL));
    real_T D_L[6][6];
    memcpy(D_L,DL1D,6*6*sizeof(real_T));
    // Snatch and map the quadratic damping matrix:
    const real_T *DQ1D = mxGetPr(ssGetSFcnParam(S,P_DQ));
    real_T D_Q[6][6];
    memcpy(D_Q,DQ1D,6*6*sizeof(real_T));
    // Counter:
    int_T i;
    
    // Compute the damping force: 
    for(i=0;i<DW_DSIZE;i++)
    {
        dw[i] = D_L[i][i]*vr[i] + D_Q[i][i]*fabs(vr[i])*vr[i];
    }
    // N.B.: This relies on the assumption of diagonal damping matrices.
}

// ************************************************************************
// coriolis_force: Function that returns the Coriolis force on the UUV.
// N.B.: For greater efficiency, a dynamic work vector is used.
// ************************************************************************
void coriolis_force(SimStruct *S)
{
    // Set a pointer to the dynamic work vector for the Coriolis force:
    real_T *dw = (real_T*) ssGetDWork(S,DW_C);
    // Set a pointer to the dynamic work vector for the relative velocity:
    real_T *vr = (real_T*) ssGetDWork(S,DW_VR);
    // Set a pointer to the continuous state vector:
    real_T *x  = ssGetContStates(S);
    
    // Snatch and map the added mass matrix:
    const real_T *MA1D = mxGetPr(ssGetSFcnParam(S,P_MA));
    real_T M_A[6][6];
    memcpy(M_A,MA1D,6*6*sizeof(real_T));
    // Snatch and map the rigid body mass matrix:
    const real_T *MB1D = mxGetPr(ssGetSFcnParam(S,P_MB));
    real_T M_B[6][6];
    memcpy(M_B,MB1D,6*6*sizeof(real_T));
    // Set a pointer to the real work vector:
    real_T *rw = ssGetRWork(S);
    // Define required variables:
    int_T i,j,k;          // counters
    real_T Av[3], Sav1[9], Sav2[9], CA[6][6];
    real_T m, Sv1[9], Sv2[9], SG[9], Iv[3], SIv[9], SGv[3][3], SvG[3][3];
    real_T CRB[6][6];
    
    // Compute M_A*v_r: 
    for(i=0;i<6;i++) {
        Av[i] = 0.0;
        for(j=0;j<6;j++) {
            Av[i] += M_A[i][j]*vr[j]; } }
    // Compute the skew symmetric matrices:
    skew_symmetric(Sav1,Av,0);
    skew_symmetric(Sav2,Av,3);
    // Assemble C_A:
    k = 0;
    for(i=0;i<3;i++) {
        for(j=0;j<3;j++) {
            CA[i][j] = 0.0;
            CA[i][j+3] = -Sav1[k];
            k++; } }
    k = 0;
    for(i=3;i<6;i++) {
        for(j=0;j<3;j++) {
            CA[i][j] = -Sav1[k];
            CA[i][j+3] = -Sav2[k];
            k++; } }
    
    // Store m for simplicity:
    m = M_B[0][0];
    // Compute Ib * v2:
    for(i=0;i<3;i++) {
        Iv[i] = 0.0;
        for(j=0;j<3;j++) {
            Iv[i] += M_B[i+3][j+3]*x[j+9]; } }
    // Compute the required skew symmetric matrices:
    skew_symmetric(Sv1,x,6);
    skew_symmetric(Sv2,x,9);
    skew_symmetric(SG,rw,RW_XG);
    skew_symmetric(SIv,Iv,0);
    // Compute S(v2)*S(G) & S(G)*S(v2):
    for(i=0;i<3;i++) {
        for(j=0;j<3;j++) {
            SvG[i][j] = 0.0;
            SGv[i][j] = 0.0;
            for(k=0;k<3;k++) {
                SvG[i][j] += Sv2[i*3+k]*SG[k*i+j];
                SGv[i][j] += SG[i*3+k]*Sv2[k*i+j]; } } }
    // Assemble C_RB:
    k = 0;
    for(i=0;i<3;i++) {
        for(j=0;j<3;j++) {
            CRB[i][j] = 0.0;
            CRB[i][j+3] = -m*Sv1[k] -m*SvG[i][j];
            k++; } }
    k = 0;
    for(i=3;i<6;i++) {
        for(j=0;j<3;j++) {
            CRB[i][j] = -m*Sv1[k] +m*SGv[i][j];
            CRB[i][j+3] = -SIv[k];
            k++; } }
    
    // Calculate the Coriolis and centripetal force:
    for(i=0;i<DW_CSIZE;i++) {
        dw[i]=0.0;
        for(j=0;j<DW_CSIZE;j++) {
            dw[i] += CA[i][j]*vr[j]+CRB[i][j]*x[j+6]; } }
}

// ************************************************************************
// transformation_matrix: Function that returns the transformation matrix.
// N.B.: For greater efficiency, a dynamic work vector is used.
// ************************************************************************
void transformation_matrix(SimStruct *S)
{
    // Set a pointer to the desired dynamic work vectors:
    real_T *dw_tt = (real_T*) ssGetDWork(S,DW_TT);
    real_T *dw_rt = (real_T*) ssGetDWork(S,DW_RT);
    // Set a pointer to the continuous state vector:
    real_T *x  = ssGetContStates(S);
    
    // Compute the translational transformation matrix: 
    dw_tt[0] = cos(x[C_PS])*cos(x[C_TH]);
    dw_tt[1] = cos(x[C_PS])*sin(x[C_PH])*sin(x[C_TH])-cos(x[C_PH])\
            *sin(x[C_PS]);
    dw_tt[2] = sin(x[C_PH])*sin(x[C_PS])+cos(x[C_PH])*cos(x[C_PS])\
            *sin(x[C_TH]);
    dw_tt[3] = cos(x[C_TH])*sin(x[C_PS]);
    dw_tt[4] = cos(x[C_PH])*cos(x[C_PS])+sin(x[C_PH])*sin(x[C_PS])\
            *sin(x[C_TH]);
    dw_tt[5] = cos(x[C_PH])*sin(x[C_PS])*sin(x[C_TH])-cos(x[C_PS])\
            *sin(x[C_PH]);
    dw_tt[6] = -sin(x[C_TH]);
    dw_tt[7] = cos(x[C_TH])*sin(x[C_PH]);
    dw_tt[8] = cos(x[C_TH])*cos(x[C_PH]);
    
    // Compute the rotational velocity transformation matrix: 
    dw_rt[0] = 1.0;
    dw_rt[1] = sin(x[C_PH])*tan(x[C_TH]);
    dw_rt[2] = cos(x[C_PH])*tan(x[C_TH]);
    dw_rt[3] = 0.0;
    dw_rt[4] = cos(x[C_PH]);
    dw_rt[5] = -sin(x[C_PH]);
    dw_rt[6] = 0.0;
    dw_rt[7] = sin(x[C_PH])/cos(x[C_TH]);
    dw_rt[8] = cos(x[C_PH])/cos(x[C_TH]);
}

// ************************************************************************
// relative_velocity: Function that returns the relative velocity vector.
// The orthonormality of the transformation matrices is exploited.
// N.B.: For greater efficiency, a dynamic work vector is used.
// ************************************************************************
void relative_velocity(SimStruct *S)
{
    int_T i,j,k;   // counters
    real_T tmp;    // temporary variable
    // Set a pointer to the desired dynamic work vectors:
    real_T *dw_tt = (real_T*) ssGetDWork(S,DW_TT);
    real_T *dw_rt = (real_T*) ssGetDWork(S,DW_RT);
    real_T *dw_vr = (real_T*) ssGetDWork(S,DW_VR);
    // Set a pointer to the current velocity vector:
    InputRealPtrsType vc = ssGetInputPortRealSignalPtrs(S,I_VC);
    // Set a pointer to the continuous state vector:
    real_T *x  = ssGetContStates(S);
    
    // Compute the current velocity in the body-fixed reference frame:
    for(i=0;i<3;i++)
    {
        tmp = 0;
        for(j=0;j<3;j++)
        {
            k = i + 3 * j;
            tmp += dw_tt[k] * (*vc[j]);
        }
        dw_vr[i] = x[i+6]-tmp;
    }
    for(i=0;i<3;i++)
    {
        tmp = 0;
        for(j=0;j<3;j++)
        {
            k = i + 3 * j;
            tmp += dw_tt[k] * (*vc[j+3]);
        }
        dw_vr[i+3] = x[i+9]-tmp;
    }
}
// ------------------------------------------------------------------------
        
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
// Check 5th parameter: P_IC
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_IC)) != 12 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_IC)) ) {
        ssSetErrorStatus(S,"5th parameter, P_IC, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 6th parameter: P_M
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_M)) != 6*6 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_M)) ) {
        ssSetErrorStatus(S,"6th parameter, P_M, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 7th parameter: P_MA
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_MA)) != 6*6 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_MA)) ) {
        ssSetErrorStatus(S,"7th parameter, P_MA, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 8th parameter: P_MB
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_MB)) != 6*6 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_MB)) ) {
        ssSetErrorStatus(S,"8th parameter, P_MB, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 9th parameter: P_DL
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_DL)) != 6*6 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_DL)) ) {
        ssSetErrorStatus(S,"9th parameter, P_DL, to S-function "
                       "\"Response Parameters\" are not dimensioned "
                       "correctly or of type double");
      return; } }
// Check 10th parameter: P_DQ
    {
    if (mxGetNumberOfElements(ssGetSFcnParam(S,P_DQ)) != 6*6 || 
            !IS_PARAM_DOUBLE(ssGetSFcnParam(S,P_DQ)) ) {
        ssSetErrorStatus(S,"10th parameter, P_DQ, to S-function "
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
    //---------------------------------------------------------------------      
    //           *** P A R A M E T E R    S E T U P ***
    //---------------------------------------------------------------------  
    //   #  Description                                Units      Dim
    //---------------------------------------------------------------------      
    //   0. Weight                                       N         1
    //   1. Buoyancy                                     N         1
    //   2. Centre of gravity                            m         3
    //   3. Centre of buoyancy                           m         3
    //   4. Initial conditions                          mix       12
    //   5. Inverse of the mass matrix                kg^(-1)     36
    //   6. Added mass matrix                            kg       36
    //   7. Rigid body mass matrix                       kg       36
    //   8. Rigid body linear damping matrix            mix       36
    //   9. Rigid body quadratic damping matrix         mix       36

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

    //---------------------------------------------------------------------        
    //         *** C O N T I N U O U S    S T A T E    S E T U P ***
    //---------------------------------------------------------------------    
    //   #                Description                 Units       Dim
    //---------------------------------------------------------------------  
    //   1.               state vector derivative      mix         12
  
    ssSetNumContStates(S,C_N);  // total number of continuous states           

    ssSetNumDiscStates(S,0); // total number of discrete states

    // Set number of input ports:
    if (!ssSetNumInputPorts(S,I_N)) return;  
  
    // Set input port widths:
    ssSetInputPortWidth(S, I_T, I_TSIZE);
    ssSetInputPortWidth(S, I_VC, I_VCSIZE);

    // If you add new inputs, you must add an element to the list below to
    // indicate if the input is used directly to compute an output.  
    ssSetInputPortDirectFeedThrough(S, I_T, NO);
    ssSetInputPortDirectFeedThrough(S, I_VC, NO);
  
    // Specify number of output ports:
    if (!ssSetNumOutputPorts(S,O_N)) return; 

    // Specify output port widths:
    ssSetOutputPortWidth(S, O_ST , O_STSIZE);
    ssSetOutputPortWidth(S, O_F , O_FSIZE);
    ssSetOutputPortWidth(S, O_VR , O_VRSIZE);

    // Set up work vectors:
    // If you need several arrays or 2D arrays of work vectors, then use
    // DWork.
    ssSetNumRWork(S, RW_N); 
    ssSetNumIWork(S, IW_N);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumDWork(S, DW_N);

    // Set up the width and type of the dynamic work vectors:
    ssSetDWorkWidth(S, DW_R, DW_RSIZE);
    ssSetDWorkWidth(S, DW_D, DW_DSIZE);
    ssSetDWorkWidth(S, DW_C, DW_CSIZE);
    ssSetDWorkWidth(S, DW_TT, DW_TTSIZE);
    ssSetDWorkWidth(S, DW_RT, DW_RTSIZE);
    ssSetDWorkWidth(S, DW_VR, DW_VRSIZE);
    ssSetDWorkDataType(S, DW_R, SS_DOUBLE);
    ssSetDWorkDataType(S, DW_D, SS_DOUBLE);
    ssSetDWorkDataType(S, DW_C, SS_DOUBLE);
    ssSetDWorkDataType(S, DW_TT, SS_DOUBLE);
    ssSetDWorkDataType(S, DW_RT, SS_DOUBLE);
    ssSetDWorkDataType(S, DW_VR, SS_DOUBLE);

    // Set up sample times:
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
    // Set a pointer to the continuous state vector:
    real_T *x  = ssGetContStates(S);

    // Set a pointer to the real work vector:
    real_T *rw = ssGetRWork(S);

    // Set a pointer to the dynamic work vectors:
    real_T *dw_r = (real_T*) ssGetDWork(S,DW_R);
    real_T *dw_d = (real_T*) ssGetDWork(S,DW_D);
    real_T *dw_c = (real_T*) ssGetDWork(S,DW_C);
    real_T *dw_tt = (real_T*) ssGetDWork(S,DW_TT);
    real_T *dw_rt = (real_T*) ssGetDWork(S,DW_RT);
    real_T *dw_vr = (real_T*) ssGetDWork(S,DW_VR);

    // Initialize counter variables:
    int i,j;
  
    // Snatch and map all the needed parameters:  
    const real_T *w = mxGetPr(ssGetSFcnParam(S,P_W));  
    const real_T *b = mxGetPr(ssGetSFcnParam(S,P_B));
    const real_T *cg = mxGetPr(ssGetSFcnParam(S,P_CG));  
    const real_T *cb = mxGetPr(ssGetSFcnParam(S,P_CB));
    const real_T *ics = mxGetPr(ssGetSFcnParam(S,P_IC));

    // Initialize the real work vector:
    rw[RW_W]  = w[0];
    rw[RW_B]  = b[0];
    rw[RW_XG] = cg[0];
    rw[RW_YG] = cg[1];
    rw[RW_ZG] = cg[2];
    rw[RW_XB] = cb[0];
    rw[RW_YB] = cb[1];
    rw[RW_ZB] = cb[2];
  
    // Initialize the dynamic work vectors:
    for (i=0;i<DW_RSIZE;i++)
    {
        dw_r[i] = 0.0;
        dw_d[i] = 0.0;
        dw_c[i] = 0.0;
    }
    for (i=0;i<DW_TTSIZE;i++)
    {
        dw_tt[i] = 0.0;
        dw_rt[i] = 0.0;
    }
    for (i=0;i<DW_VRSIZE;i++)
    {
        dw_vr[i] = 0.0;
    }
  
    // Debugging:
//     for(i=0;i<RW_N;i++) printf("%f\n", rw[i]);
//     for(i=0;i<C_N;i++) printf("%f\n", ics[i]);
//     const real_T *M1D = mxGetPr(ssGetSFcnParam(S,P_M));
//     real_T M_inv[6][6];
//     memcpy(M_inv,M1D,6*6*sizeof(real_T));
//     for(i=0;i<6;i++){
//         for(j=0;j<6;j++) printf("%f\t", M_inv[i][j]);
//         printf("\n");}
//     const real_T *DL1D = mxGetPr(ssGetSFcnParam(S,P_DL));
//     real_T D_L[6][6];
//     memcpy(D_L,DL1D,6*6*sizeof(real_T));
//     for(i=0;i<6;i++){
//     for(j=0;j<6;j++) printf("%f\t", D_L[i][j]);
//     printf("\n");}
//     const real_T *DQ1D = mxGetPr(ssGetSFcnParam(S,P_DQ));
//     real_T D_Q[6][6];
//     memcpy(D_Q,DQ1D,6*6*sizeof(real_T));
//     for(i=0;i<6;i++){
//     for(j=0;j<6;j++) printf("%f\t", D_Q[i][j]);
//     printf("\n");}
//     real_T v[3]={1.,2.,3.}, M[9];
//     int_T k;
//     skew_symmetric(M,v,0);
//     k = 0;
//     for(i=0;i<3;i++) {
//         for(j=0;j<3;j++) {
//             printf("%f\t", M[k]);
//             k++; }
//         printf("\n");}
  
    // Initialize the state vector:
    for(i=0;i<C_N;i++) x[i]=ics[i];

    ssSetSimStateCompliance(S,USE_DEFAULT_SIM_STATE);
}
#endif

// ************************************************************************
// mdlOutputs: Calc outputs at the start of each major integration step
// ************************************************************************
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // set a pointers to the outputs
    real_T *ySt   = ssGetOutputPortSignal(S,O_ST);  // states
    real_T *yF    = ssGetOutputPortSignal(S,O_F);   // forces
    real_T *yVr   = ssGetOutputPortSignal(S,O_VR);  // relative velocity
    
    // set a pointer to the continous state vector
    real_T *x  = ssGetContStates(S);
  
    // Set a pointer to the dynamic work vectors:
    real_T *dw_r = (real_T*) ssGetDWork(S,DW_R);
    real_T *dw_d = (real_T*) ssGetDWork(S,DW_D);
    real_T *dw_c = (real_T*) ssGetDWork(S,DW_C);
    real_T *dw_vr = (real_T*) ssGetDWork(S,DW_VR);
  
    // Toss continuous states to the output port:
    int_T i;   // counter
    for(i=0;i<O_STSIZE;i++) ySt[i] = x[i];
    
    // Output the forces acting on the UUV:
    for(i=0;i<DW_RSIZE;i++) yF[i] = dw_r[i];
    for(i=0;i<DW_DSIZE;i++) yF[i+DW_RSIZE] = dw_d[i];
    for(i=0;i<DW_CSIZE;i++) yF[i+DW_RSIZE+DW_DSIZE] = dw_c[i];
    
    // Output the relative velocity vector:
    for(i=0;i<DW_VRSIZE;i++) yVr[i] = dw_vr[i];
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
  int_T i,j,k;   // counters
  real_T tmp;    // temporary value
  
  real_T *dx = ssGetdX(S);         // ptr to right side of x' = f(x,u,t)
  
  // Set a pointer to the thrust vector:
  InputRealPtrsType thrust = ssGetInputPortRealSignalPtrs(S,I_T);
  
  // Compute the transformation matrix:
  transformation_matrix(S);
  real_T *dw_tt = (real_T*) ssGetDWork(S,DW_TT);
  real_T *dw_rt = (real_T*) ssGetDWork(S,DW_RT);
  
  // Compute the relative velocity:
  relative_velocity(S);
  real_T *dw_vr = (real_T*) ssGetDWork(S,DW_VR);
  
  // Snatch and map the inverse mass matrix: 
  const real_T *M1D = mxGetPr(ssGetSFcnParam(S,P_M));
  real_T M_inv[6][6];
  memcpy(M_inv,M1D,6*6*sizeof(real_T));
  
//   // Snatch and map the rigid body linear damping matrix: 
//   const real_T *DL1D = mxGetPr(ssGetSFcnParam(S,P_DL));
//   real_T D_L[6][6];
//   memcpy(D_L,DL1D,6*6*sizeof(real_T));
//   
//   // Snatch and map the rigid body quadratic damping matrix: 
//   const real_T *DQ1D = mxGetPr(ssGetSFcnParam(S,P_DQ));
//   real_T D_Q[6][6];
//   memcpy(D_Q,DQ1D,6*6*sizeof(real_T));
  
  // Compute the restoring force:
  restoring_force(S);
  real_T *dw_r = (real_T*) ssGetDWork(S,DW_R);
  
  // Compute the damping force:
  damping_force(S);
  real_T *dw_d = (real_T*) ssGetDWork(S,DW_D);
  
  // Compute the Coriolis force force:
  coriolis_force(S);
  real_T *dw_c = (real_T*) ssGetDWork(S,DW_C);
  
  // Compute the state derivatives:
  k = 0;
  for (i=0;i<3;i++){
      tmp = 0.0;
      for (j=0;j<3;j++){
          tmp += dw_tt[k]*dw_vr[j];
          k++;
      }
      dx[i] = tmp;
  }
  k = 0;
  for (i=3;i<6;i++){
      tmp = 0.0;
      for (j=0;j<3;j++){
          tmp += dw_rt[k]*dw_vr[j+3];
          k++;
      }
      dx[i] = tmp;
  }
  for (i=0;i<6;i++){
      tmp = 0.0;
      for (j=0;j<6;j++)
          tmp += M_inv[i][j] * (*thrust[j]-dw_r[j]-dw_d[j]-dw_c[j]);
      dx[i+6] = tmp;
  }
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