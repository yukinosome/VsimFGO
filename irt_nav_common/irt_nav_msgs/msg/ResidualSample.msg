uint64   id

uint8    GROUND_TRUTH=0
uint8    ESTIMATE=1
uint8    SAMPLED_FROM_GROUND_TRUTH=2
uint8    SAMPLED_FROM_ESTIMATE=3

uint8    type

float64[]   whitened_error
float64[]   unwhitened_error
float64     loss_error
float64     noise_model_weight