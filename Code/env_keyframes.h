#ifndef ENV_KEYFRAMES
#define ENV_KEYFRAMES

// ENVELOPE KEYFRAMES:
//    - 1ms rise, 2ms fall
//    - 1ms rise, 100ms fall
//    - 1ms rise, 800ms fall
//    - 100ms rise, 667ms fall
//    - 400ms rise, 400ms fall
//    - 667ms rise, 100ms fall
//    - 800ms rise, 1ms fall
//    - 100ms rise, 1ms fall
//    - 2ms rise, 1ms fall

const unsigned int atk_incs[] = {
    42949673,
    42949673,
    42949673,
    429497,
    85899,
    64425,
    53687,
    429497,
    21474836
};

const unsigned int decay_incs[] = {
    21474836,
    429497,
    53687,
    64425,
    85899,
    429497,
    42949673,
    42949673,
    42949673
};

const unsigned int num_keyframes = 9;

#endif