#include <math.h>
#include <stdio.h>

#include "xiao_getup/policy_network.h"

int main(void) {
    const float obs[OBS_DIM] = {0.5f, -0.2f, 0.1f, -0.3f};
    float action[ACTION_DIM];
    forward_policy(obs, action);
    const float expected[2] = {-0.50096828f, -0.71039355f};
    if (fabsf(action[0] - expected[0]) > 1e-5f || fabsf(action[1] - expected[1]) > 1e-5f) {
        fprintf(stderr, "policy mismatch %f %f\n", action[0], action[1]);
        return 1;
    }
    puts("ok");
    return 0;
}
