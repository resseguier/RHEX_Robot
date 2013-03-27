// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/*!
 * @file  RHEXPD.cpp
 * @brief RHEX PD component
 * $Date$
 *
 * $Id$
 */

#include "RHEXPD.h"

#include <iostream>

#define TIMESTEP 0.001

#define ANGLE_FILE "/tmp/RHEX.pos"
#define VEL_FILE   "/tmp/RHEX.vel"
// #define ACC_FILE   "etc/acc.dat"

// #define GAIN_FILE  "etc/PDgain.dat"

namespace {
  const bool CONTROLLER_BRIDGE_DEBUG = false;
}


// Module specification
// <rtc-template block="module_spec">
static const char* samplepd_spec[] =
  {
    "implementation_id", "RHEXPD",
    "type_name",         "RHEXPD",
    "description",       "RHEX PD component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables

    ""
  };
// </rtc-template>

RHEXPD::RHEXPD(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_angleIn("angle", m_angle),
    m_torqueOut("torque", m_torque),
    
    // </rtc-template>
    dummy(0),
    qold(DOF),
    error(DOF)
{
  if( CONTROLLER_BRIDGE_DEBUG )
  {
    std::cout << "RHEXPD::RHEXPD" << std::endl;
  }

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>
  FP.open("/tmp/torque.log");
  FPC.open("/tmp/torque_by_dq.log");
  count_before_start = 0;
  for(int i=0;i<DOF;i++)
	error[i] = 0;
}

RHEXPD::~RHEXPD()
{
   FP.close();
   FPC.close();
  closeFiles();
//   delete [] Pgain;
//   delete [] Dgain;
}


RTC::ReturnCode_t RHEXPD::onInitialize()
{
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  if( CONTROLLER_BRIDGE_DEBUG )
  {
    std::cout << "onInitialize" << std::endl;
  }

  // Set InPort buffers
  addInPort("angle", m_angleIn);
  
  // Set OutPort buffer
  addOutPort("torque", m_torqueOut);
  // </rtc-template>
/*
  Pgain = new double[DOF];
  Dgain = new double[DOF];

  gain.open(GAIN_FILE);
  if (gain.is_open()){
    for (int i=0; i<DOF; i++){
      gain >> Pgain[i];
      gain >> Dgain[i];
    }
    gain.close();
  }else{
    std::cerr << GAIN_FILE << " not found" << std::endl;
  }*/
  m_torque.data.length(DOF);
  m_angle.data.length(DOF);

  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t RHEXPD::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RHEXPD::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RHEXPD::onShutdown(RTC::UniqueId ec_id)
{
    log("RHEXPD::onShutdown");
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t RHEXPD::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "on Activated" << std::endl;
  openFiles();
  
  if(m_angleIn.isNew()){
    m_angleIn.read();
  }
  
  for(int i=0; i < DOF; ++i){
    qold[i] = m_angle.data[i];
    q_ref[i] = dq_ref[i] = 0.0;
    
  }
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t RHEXPD::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "on Deactivated" << std::endl;
  closeFiles();
  return RTC::RTC_OK;
}



RTC::ReturnCode_t RHEXPD::onExecute(RTC::UniqueId ec_id)
{
  if( CONTROLLER_BRIDGE_DEBUG )
  {
    std::cout << "RHEXCOntroller onExecute" << std::endl;
    std::string localStr;
    std::cin >> localStr; 
  }

  if(m_angleIn.isNew()) //   && ( count_before_start > 50  || count_before_start == 0))
  {
    m_angleIn.read();
  }
//   count_before_start=1;

  if(!angle.eof()){
    angle >> q_ref[0]; 
    vel >> dq_ref[0];// skip time
    for (int i=0; i<DOF; i++){
      angle >> q_ref[i];
      vel >> dq_ref[i];
//       q_ref[i] = 0.0;
//       dq_ref[i] = 0.0;

    }
  }


  for(int i=0; i<DOF; i++){
    double q = m_angle.data[i];
    double dq = (q - qold[i]) / TIMESTEP;
    qold[i] = q;
    
    error[i] += q - q_ref[i];
    

//       std::cout<<" q_ref("<<i<<" = "<< q_ref[i] <<std::endl;
//     double tau = -(q - q_ref[i]) * Pgain[i] - (dq - dq_ref[i]) * Dgain[i] ; //- error[i] * 100;
//       double tau = -(q - q_ref[i]) * 40.0 - (dq - dq_ref[i]) * 20.0 ; //- error[i] * 100;
    
    double tau;
    if (i < 24 )
          tau = -(q  - q_ref[i]) * 300.0 - (dq - dq_ref[i]) * 10.0 - error[i] * 0.0;
    else if (i > 23)
	  tau = -(q  - q_ref[i]) * 10.0 - (dq - dq_ref[i]) * 1.0;
    else
	  tau = -(q  ) * 300.0 - (dq ) * 10.0 ; //- error[i] * 10;

//     double tau = -(q ) * 100.0;
    
//     if ( i %4 == 0)
//     {
// 	if (tau < -12)	tau = -12;
// 	if (tau >  12)	tau =  12;
//     }
//     
//     if ( i %4 == 1)
//     {
// 	if (tau < -12)	tau = -12;
// 	if (tau >  12)	tau =  12;
//     }
//       
    double satur = 45.0;
    
    if (tau < -satur)	tau = -satur;
    if (tau >  satur)	tau =  satur;
    
    m_torque.data[i] = tau;
    FPC<<  tau * dq<<" ";    
    FP<<  tau<<" ";
  }
  FP<<std::endl;
  FPC<<std::endl;
      
  m_torqueOut.write();
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t RHEXPD::onAborting(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RHEXPD::onError(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RHEXPD::onReset(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RHEXPD::onStateUpdate(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t RHEXPD::onRateChanged(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}
*/

void RHEXPD::openFiles()
{
  angle.open(ANGLE_FILE);
  if(!angle.is_open()){
    std::cerr << ANGLE_FILE << " not opened" << std::endl;
  }

  vel.open(VEL_FILE);
  if (!vel.is_open()){
    std::cerr << VEL_FILE << " not opened" << std::endl;
  }  
}

void RHEXPD::closeFiles()
{
  if( angle.is_open() ){
    angle.close();
    angle.clear();
  }
  if( vel.is_open() ){
    vel.close();
    vel.clear();
  }
}

extern "C"
{

  DLL_EXPORT void RHEXPDInit(RTC::Manager* manager)
  {
    coil::Properties profile(samplepd_spec);
    manager->registerFactory(profile,
                             RTC::Create<RHEXPD>,
                             RTC::Delete<RHEXPD>);
  }

};

