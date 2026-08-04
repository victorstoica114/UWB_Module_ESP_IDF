#ifndef UWB_NATIVE_DS_RUNTIME_H
#define UWB_NATIVE_DS_RUNTIME_H

#include "esp_err.h"
#include "uwb_native_ds_twr.h"

/*
 * Owns the complete Native DS-TWR protocol-side runtime: position solver,
 * protocol telemetry and the clean three-packet DS-TWR engine.  The caller
 * supplies only DW3000/OS primitives through radio_backend.
 */
esp_err_t uwb_native_ds_runtime_run(
    const struct uwb_native_ds_config *config,
    const struct uwb_native_ds_radio_ops *radio_backend);

#endif
