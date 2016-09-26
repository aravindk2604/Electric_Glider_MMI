/**
*  ***********************************************************************
*  @file    app_cfg.h
*  @author  
*  @version V1.00
*  @brief   Application Specific Configuration Module  
*  @date    14/05/2014
**************************************************************************
*/

#ifndef  __APP_CFG_H__
#define  __APP_CFG_H__

/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/

//#define  uC_PROBE_OS_PLUGIN              DEF_ENABLED            /* DEF_ENABLED = Present, DEF_DISABLED = Not Present        */
//#define  uC_PROBE_COM_MODULE             DEF_ENABLED

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_TASK_START_PRIO                  25			   //!<	APP_TASK_START_PRIORITY	
#define  APP_TASK_KBD_PRIO                    26			   //!<	APP_TASK_KBD_PRIORITY
#define  APP_TASK_PROBE_STR_PRIO              27			   //!<	APP_TASK_PROBE_STR_PRIORITY
															   
#define  OS_PROBE_TASK_PRIO                   28			   //!<	OS_PROBE_TASK_PRIORITY
#define  OS_PROBE_TASK_ID                     28			   //!<	OS_PROBE_TASK_ID

#define  OS_TASK_TMR_PRIO              (OS_LOWEST_PRIO - 2)	   //!<	OS_TASK_TMR_PRIORITY

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*                            Size of the task stacks (# of OS_STK entries)
*********************************************************************************************************
*/

#define  APP_TASK_START_STK_SIZE             256				//!< Size of the task stacks
#define  APP_TASK_KBD_STK_SIZE               256				//!< Size of the task stacks
#define  APP_TASK_PROBE_STR_STK_SIZE         512				//!< Size of the task stacks

#define  OS_PROBE_TASK_STK_SIZE              512				//!< Size of the task stacks

/*
*********************************************************************************************************
*                               uC/Probe plug-in for uC/OS-II CONFIGURATION
*********************************************************************************************************
*/

//#define  OS_PROBE_TASK                         0                //!< Task will be created for uC/Probe OS Plug-In             */
//#define  OS_PROBE_TMR_32_BITS                  0                //!< uC/Probe OS Plug-In timer is a 32-bit timer              */
//#define  OS_PROBE_HOOKS_EN                     0                //!< Hooks to update OS_TCB profiling members will be included*/

/*
*********************************************************************************************************
*                                      uC/OS-II DCC CONFIGURATION
*********************************************************************************************************
*/
																   
#define  OS_CPU_ARM_DCC_EN                     0				    //!< uC/OS-II DCC CONFIGURATION


/*
*********************************************************************************************************
*                                     TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#define  TRACE_LEVEL_OFF                       0				 //!< TRACE CONFIGURATION OFF 
#define  TRACE_LEVEL_INFO                      1				 //!< TRACE CONFIGURATION INFO
#define  TRACE_LEVEL_DEBUG                     2				 //!< DEBUG CONFIGURATION 
																 
#define  APP_TRACE_LEVEL                TRACE_LEVEL_DEBUG		 //!< TRACE CONFIGURATION LEVEL
#define  APP_TRACE                        						 //!< TRACE

#define  APP_TRACE_INFO(x)            ((APP_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_TRACE x) : (void)0)		 //!< TRACE / DEBUG CONFIGURATION
#define  APP_TRACE_DEBUG(x)           ((APP_TRACE_LEVEL >= TRACE_LEVEL_DEBUG) ? (void)(APP_TRACE x) : (void)0)		 //!< TRACE / DEBUG CONFIGURATION

#endif
