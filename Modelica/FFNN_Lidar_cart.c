#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GRID_SIZE 27
#define TARGET_SIZE 3
#define STATE_SIZE 3
#define INPUT_SIZE (GRID_SIZE + TARGET_SIZE + STATE_SIZE) 
#define HIDDEN_SIZE 16
#define OUTPUT_SIZE 3

void compute_ffnn_action_v2(const double* grid, 
                            const double* target_p_rel_local,
                            double vx_local, double vy_local, double vz_local, 
                            const double* theta_weights, 
                            double* safe_vx_local, double* safe_vy_local, double* safe_vz_local) {
    
    double inputs[INPUT_SIZE]; 
    
    for (int i = 0; i < GRID_SIZE; i++) {
        inputs[i] = grid[i];
    }

    inputs[GRID_SIZE + 0] = tanh(target_p_rel_local[0] / 25.0);
    inputs[GRID_SIZE + 1] = tanh(target_p_rel_local[1] / 25.0);
    inputs[GRID_SIZE + 2] = tanh(target_p_rel_local[2] / 25.0);

    inputs[GRID_SIZE + TARGET_SIZE + 0] = vx_local / 3.0;
    inputs[GRID_SIZE + TARGET_SIZE + 1] = vy_local / 3.0;
    inputs[GRID_SIZE + TARGET_SIZE + 2] = vz_local / 3.0;

    double hidden[HIDDEN_SIZE];
    int w1_start = 0;
    int b1_start = INPUT_SIZE * HIDDEN_SIZE;
    
    for (int i = 0; i < HIDDEN_SIZE; i++) {
        double sum = 0.0;
        for (int j = 0; j < INPUT_SIZE; j++) {
            sum += inputs[j] * theta_weights[w1_start + (i * INPUT_SIZE) + j];
        }
        sum += theta_weights[b1_start + i];
        hidden[i] = (sum > 0.0) ? sum : 0.0;
    }

    double outputs[OUTPUT_SIZE];
    int w2_start = b1_start + HIDDEN_SIZE;                 
    int b2_start = w2_start + (OUTPUT_SIZE * HIDDEN_SIZE); 
    
    for (int i = 0; i < OUTPUT_SIZE; i++) {
        double sum = 0.0;
        for (int j = 0; j < HIDDEN_SIZE; j++) {
            sum += hidden[j] * theta_weights[w2_start + (i * HIDDEN_SIZE) + j];
        }
        sum += theta_weights[b2_start + i];
        outputs[i] = sum;
    }

    *safe_vx_local = 3.0 * tanh(outputs[0]); 
    *safe_vy_local = 3.0 * tanh(outputs[1]); 
    *safe_vz_local = 3.0 * tanh(outputs[2]); 
}
