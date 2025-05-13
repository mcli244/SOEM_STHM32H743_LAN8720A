/*
 * Simple Open EtherCAT Master Library 
 *
 * File    : ethercatconfiglist.h
 * Version : 1.3.0
 * Date    : 24-02-2013
 * Copyright (C) 2005-2013 Speciaal Machinefabriek Ketels v.o.f.
 * Copyright (C) 2005-2013 Arthur Ketels
 * Copyright (C) 2008-2009 TU/e Technische Universiteit Eindhoven 
 *
 * SOEM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the Free
 * Software Foundation.
 *
 * SOEM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * As a special exception, if other files instantiate templates or use macros
 * or inline functions from this file, or you compile this file and link it
 * with other works to produce a work based on this file, this file does not
 * by itself cause the resulting work to be covered by the GNU General Public
 * License. However the source code for this file must still be made available
 * in accordance with section (3) of the GNU General Public License.
 *
 * This exception does not invalidate any other reasons why a work based on
 * this file might be covered by the GNU General Public License.
 *
 * The EtherCAT Technology, the trade name and logo “EtherCAT�? are the intellectual
 * property of, and protected by Beckhoff Automation GmbH. You can use SOEM for
 * the sole purpose of creating, using and/or selling or otherwise distributing
 * an EtherCAT network master provided that an EtherCAT Master License is obtained
 * from Beckhoff Automation GmbH.
 *
 * In case you did not receive a copy of the EtherCAT Master License along with
 * SOEM write to Beckhoff Automation GmbH, Eiserstraße 5, D-33415 Verl, Germany
 * (www.beckhoff.com).
 */

/** \file
 * \brief
 * DEPRICATED Configuration list of known EtherCAT slave devices.
 *
 * If a slave is found in this list it is configured according to the parameters
 * in the list. Otherwise the configuration info is read directly from the slave
 * EEPROM (SII or Slave Information Interface).
 */
/*
   explanation of dev:                                   
    1: static device with no IO mapping ie EK1100        
    2: input device no mailbox ie simple IO device       
    3: output device no mailbox                          
    4: input device with mailbox configuration           
    5: output device with mailbox configuration          
    6: input/output device no mailbox                    
    7: input.output device with mailbox configuration           
*/
#define EC_CONFIGEND 0xffffffff

ec_configlist_t ec_configlist[] = {
	{/*Man=*/0x00000002,/*ID=*/0x13ed3052,/*Name=*/"EL5101"    ,/*dtype=*/7,/*Ibits=*/40,/*Obits=*/24,/*SM2a*/0x1000,/*SM2f*/0x00010024,/*SM3a*/0x1100,/*SM3f*/0x00010020,/*FM0ac*/1,/*FM1ac*/1,NULL,NULL},
	{/*Man=*/0x00004321,/*ID=*/0x00008500,/*Name=*/"DM3-ECA882",/*dtype=*/7,/*Ibits=*/96,/*Obits=*/48,/*SM2a*/0x1200,/*SM2f*/0x00010064,/*SM3a*/0x1400,/*SM3f*/0x00010020,/*FM0ac*/1,/*FM1ac*/1,NULL,NULL},//ecat_dm3_eca882_init,ecat_dm3_eca882_read_write},  
	{/*Man=*/EC_CONFIGEND,/*ID=*/0x00000000,/*Name=*/"" 	   ,/*dtype=*/0,/*Ibits=*/ 0,/*Obits=*/ 0,/*SM2a*/	   0,/*SM2f*/		  0,/*SM3a*/	 0,/*SM3f*/ 		0,/*FM0ac*/0,/*FM1ac*/0,NULL,NULL}
};
