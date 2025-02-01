
#ifndef __DMAC_WMM_TEST_H__
#define __DMAC_WMM_TEST_H__

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif


/*****************************************************************************
  1 ??????????????
*****************************************************************************/
#include "oal_ext_if.h"
#include "frw_ext_if.h"
#include "hal_ext_if.h"
#include "dmac_ext_if.h"
#include "dmac_vap.h"
#ifdef _PRE_WLAN_ALG_ENABLE
#include "alg_ext_if.h"
#endif
#include "dmac_test_main.h"

#undef  THIS_FILE_ID
#define THIS_FILE_ID OAM_FILE_ID_DMAC_WMM_TEST_H

/*****************************************************************************
  2 ??????
*****************************************************************************/
#define DMAC_TEST_WMM_SUSPEND       BIT1
#define DMAC_TEST_WMM_ENABLE        BIT0

/*****************************************************************************
  3 ????????
*****************************************************************************/


/*****************************************************************************
  4 ????????????
*****************************************************************************/

/*****************************************************************************
  5 ??????????
*****************************************************************************/


/*****************************************************************************
  6 ????????
*****************************************************************************/


/*****************************************************************************
  7 STRUCT????
*****************************************************************************/


/*****************************************************************************
  8 UNION????
*****************************************************************************/


/*****************************************************************************
  9 OTHERS????
*****************************************************************************/


/*****************************************************************************
  10 ????????
*****************************************************************************/
extern oal_uint32  dmac_test_open_wmm_test(mac_vap_stru *pst_mac_vap, oal_uint8 uc_test_type);
extern oal_void  dmac_config_set_wmm_open_cfg(hal_to_dmac_vap_stru *pst_hal_vap, mac_wme_param_stru  *pst_wmm);
extern oal_void  dmac_config_set_wmm_close_cfg(hal_to_dmac_vap_stru *pst_hal_vap, mac_wme_param_stru  *pst_wmm);


#ifdef __cplusplus
    #if __cplusplus
        }
    #endif
#endif

#endif /* end of __DMAC_WMM_TEST_H__ */

