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

int pid_calculate(struct pid *p, float input)
{
    int dt;
    float error;
    struct timeb now;

    /* calculate dt - in miliseconds */
    ftime(&now);
    dt = (float) (
        1000.0 *
        (now.time - p->last_updated.time) +
        (now.millitm - p->last_updated.millitm)
    );

    /* calculate output */
    if (dt >= p->sample_rate) {
        /* calculate errors */
        error = p->setpoint - input;
        if (fabs(error) > p->dead_zone) {
            p->sum_error += error * (float) (dt / 1000.0);
        }

        /* calculate output */
        p->output = (p->k_p * error);
        p->output += (p->k_i * p->sum_error);
        p->output -= (p->k_d * (input - p->prev_error));

        /* limit boundaries */
        if (p->output > p->max) {
            p->output = p->max;
        } else if (p->output < p->min) {
            p->output = p->min;
        }

        /* update error and last_updated */
        p->prev_error = error;
        ftime(&p->last_updated);
    }

    return 0;
}
