#ifndef UWB_LOCALIZATION_SOLVER_H
#define UWB_LOCALIZATION_SOLVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_LOCALIZATION_MAX_OBSERVATIONS 64U

enum uwb_localization_measurement_type {
    UWB_LOCALIZATION_MEASUREMENT_RANGE = 0,
    UWB_LOCALIZATION_MEASUREMENT_RANGE_DIFFERENCE,
};

enum uwb_localization_robust_loss {
    UWB_LOCALIZATION_LOSS_NONE = 0,
    UWB_LOCALIZATION_LOSS_HUBER,
    UWB_LOCALIZATION_LOSS_CAUCHY,
};

/*
 * RANGE:
 *   measured_m = distance(tag, first)
 *
 * RANGE_DIFFERENCE:
 *   measured_m = distance(tag, second) - distance(tag, first)
 *
 * Every observation belongs to one radio frame/batch. Robust weighting is
 * therefore spatial/per-frame and never averages positions over time.
 */
struct uwb_localization_observation {
    enum uwb_localization_measurement_type type;
    double first_x_m;
    double first_y_m;
    double second_x_m;
    double second_y_m;
    double measured_m;
    double base_weight;
};

struct uwb_localization_seed {
    bool valid;
    double x_m;
    double y_m;
};

struct uwb_localization_solver_config {
    enum uwb_localization_robust_loss loss;
    double loss_scale_m;
    double minimum_distance_m;
    double step_tolerance_m;
    double maximum_step_m;
    double initial_damping;
    uint8_t maximum_iterations;
};

struct uwb_localization_result {
    bool valid;
    double x_m;
    double y_m;
    double residual_rms_m;
    double weighted_residual_rms_m;
    double maximum_abs_residual_m;
    double sigma_m;
    double geometry_hdop;
    double normal_condition_number;
    uint8_t observation_count;
    uint8_t downweighted_count;
    uint8_t iterations;
};

struct uwb_localization_solver_config
uwb_localization_solver_default_config(void);

bool uwb_localization_solve_2d(
    const struct uwb_localization_observation *observations,
    size_t observation_count,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result);

/*
 * Generalized least-squares variant for correlated observations.  The
 * precision matrix is the inverse measurement covariance, in row-major
 * order, and must contain observation_count x observation_count elements.
 * base_weight and the robust loss are applied as symmetric row/column
 * scaling around this matrix (D * precision * D).  This preserves Passive
 * DS-TWR's same-star timestamp covariance while still limiting a bad radio
 * measurement inside the current batch.
 */
bool uwb_localization_solve_2d_correlated(
    const struct uwb_localization_observation *observations,
    size_t observation_count,
    const double *precision_matrix,
    size_t precision_row_stride,
    const struct uwb_localization_seed *seed,
    const struct uwb_localization_solver_config *config,
    struct uwb_localization_result *result);

#ifdef __cplusplus
}
#endif

#endif /* UWB_LOCALIZATION_SOLVER_H */
