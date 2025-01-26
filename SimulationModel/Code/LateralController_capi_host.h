#ifndef LateralController_cap_host_h__
#define LateralController_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"

typedef struct {
  rtwCAPI_ModelMappingInfo mmi;
} LateralController_host_DataMapInfo_T;

#ifdef __cplusplus

extern "C"
{

#endif

  void LateralController_host_InitializeDataMapInfo
    (LateralController_host_DataMapInfo_T *dataMap, const char *path);

#ifdef __cplusplus

}

#endif
#endif                                 /* HOST_CAPI_BUILD */
#endif                                 /* LateralController_cap_host_h__ */

/* EOF: LateralController_capi_host.h */
