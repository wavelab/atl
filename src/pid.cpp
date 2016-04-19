#include "awesomo/pid.hpp"


struct pid *pid_setup(struct pid_config *c)
{
    struct pid *p;

    p = (struct pid *) malloc(sizeof(struct pid));

    p->sample_rate = (c->sample_rate == NAN) ? 100 : c->sample_rate;

    p->setpoint = (c->setpoint == NAN) ? 0 : c->setpoint;
    p->output = 0.0f;

    p->prev_error = 0.0f;
    p->sum_error = 0.0f;

    p->k_p = (c->k_p == NAN) ? 0.0f : c->k_p;
    p->k_i = (c->k_i == NAN) ? 0.0f : c->k_i;
    p->k_d = (c->k_d == NAN) ? 0.0f : c->k_d;

    p->dead_zone = c->dead_zone;
    p->min = c->min;
    p->max = c->max;
    ftime(&p->last_updated);

    return p;
}

void pid_destroy(void *target)
{
    struct pid *p;

    p = (struct pid *) target;
    free(p);
    p = NULL;
}

int pid_calculate(struct pid *p, float input, ros::Duration dt)
{
    float error;

    // calculate errors
    error = p->setpoint - input;
    if (fabs(error) > p->dead_zone) {
        p->sum_error += error * dt.toSec();
    }

    // calculate output
    p->p_error = p->k_p * error;
    p->i_error = p->k_i * p->sum_error;
    p->d_error = p->k_d * (error - p->prev_error);
    p->output = p->p_error + p->i_error + p->d_error;

    // limit boundaries
    if (p->output > p->max) {
        p->output = p->max;
    } else if (p->output < p->min) {
        p->output = p->min;
    }

    // update error
    p->prev_error = error;

    return 0;
}
