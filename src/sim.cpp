#include "awesomo/sim.hpp"


void sim_record_step(FILE *out_file, struct quadrotor *q)
{
    char buf[9046];

    // setup
    memset(buf, '\0', 9046);

    // build string
    // position
    sprintf(buf, "%f,", q->position(0));
    sprintf(strlen(buf) + buf, "%f,", q->position(1));
    sprintf(strlen(buf) + buf, "%f,", q->position(2));
    // orientation
    sprintf(strlen(buf) + buf, "%f,", q->orientation(0));
    sprintf(strlen(buf) + buf, "%f,", q->orientation(1));
    sprintf(strlen(buf) + buf, "%f", q->orientation(2));

    // write to file
    fprintf(out_file, "%s\n", buf);
}

void sim_loop(void)
{
    int i;
    struct quadrotor q;
    struct world w;
    FILE *output;

    // setup world
    w.dt = 0.1f;
    w.gravity = 10.0f;

    // setup quadcopter
    quadrotor_setup(&q);

    q.orientation << deg2rad(0.0f), deg2rad(0.0f), deg2rad(0.0f);
    q.position << 0.0f, 0.0f, 0.0f;
    q.velocity << 0.0f, 0.0f, 0.0f;
    q.acceleration << 0.0f, 0.0f, 0.0f;
    q.rotors << 1.9f, 2.0f, 1.9f, 2.0f;

    q.m = 1.0f;
    q.L = 1.0f;
    q.k = 1.0f;
    q.b = 1.0f;
    q.kd = 0.2f;
    quadrotor_inertia_matrix(&q, 2.0f, 2.0f, 2.0f);

    // open output file
    output = fopen("/tmp/sim.out", "w");

    // loop
    for (i = 0; i < 500; i++) {
        quadrotor_update(&q, w.gravity, w.dt);

        // record
        sim_record_step(output, &q);

        // terminator
        if (q.position(2) < -1.0f) {
            break;
        } else if (q.position(2) > 20.0f) {
            break;
        }
    }

    // clean up
    fclose(output);
}
