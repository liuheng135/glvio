#include "rotation.h"

void rotate3(float val[3],enum Rotation rotation)
{
    float  tmp;
    switch (rotation) {
	case ROTATION_NONE:
        return;
    case ROTATION_YAW_90: {
        tmp = val[0]; val[0] = -val[1];val[1] = tmp;
        return;
    }
    case ROTATION_YAW_180:
        val[0] = -val[0]; val[1] = -val[1];
        return;
    case ROTATION_YAW_270: {
        tmp = val[0]; val[0] = val[1]; val[1] = -tmp;
        return;
    }
    case ROTATION_ROLL_180: {
        val[1] = -val[1]; val[2] = -val[2];
        return;
    }
    case ROTATION_ROLL_180_YAW_90: {
        tmp = val[0]; val[0] = val[1]; val[1] = tmp; val[2] = -val[2];
        return;
    }
    case ROTATION_PITCH_180: {
        val[0] = -val[0]; val[2] = -val[2];
        return;
    }
    case ROTATION_ROLL_180_YAW_270: {
        tmp = val[0]; val[0] = -val[1]; val[1] = -tmp; val[2] = -val[2];
        return;
    }
    case ROTATION_ROLL_90: {
        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_90: {
        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
        tmp = val[0]; val[0] = -val[1]; val[1] = tmp;
        return;
    }
    case ROTATION_ROLL_270: {
        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_90: {
        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
        tmp = val[0]; val[0] = -val[1]; val[1] = tmp;
        return;
    }
    case ROTATION_PITCH_90: {
        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
        return;
    }
    case ROTATION_PITCH_270: {
        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_90: {
        val[2] = -val[2];
        tmp = -val[0]; val[0] = -val[1]; val[1] = tmp;
        return;
    }
    case ROTATION_PITCH_180_YAW_270: {
        val[0] = -val[0]; val[2] = -val[2];
        tmp = val[0]; val[0] = val[1]; val[1] = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_90: {
        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_90: {
        val[1] = -val[1]; val[2] = -val[2];
        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_90: {
        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
        tmp = val[2]; val[2] = -val[0]; val[0] = tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180: {
        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
        val[0] = -val[0]; val[2] = -val[2];
        return;
    }
    case ROTATION_ROLL_270_PITCH_180: {
        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
        val[0] = -val[0]; val[2] = -val[2];
        return;
    }
    case ROTATION_ROLL_90_PITCH_270: {
        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_270: {
        val[1] = -val[1]; val[2] = -val[2];
        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_270: {
        tmp = val[2]; val[2] = -val[1]; val[1] = tmp;
        tmp = val[2]; val[2] = val[0]; val[0] = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180_YAW_90: {
        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
        val[0] = -val[0]; val[2] = -val[2];
        tmp = val[0]; val[0] = -val[1]; val[1] = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_270: {
        tmp = val[2]; val[2] = val[1]; val[1] = -tmp;
        tmp = val[0]; val[0] = val[1]; val[1] = -tmp;
        return;
    }
    }
}

