  
/* Copyright (c) 2012-2015 Stanislaw Halik
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */
#pragma once

#include <QMutex>
#include <QTimer>
#include <accela-settings.hpp>
#include <fagx_datatype.h>

enum Axis : int
{
    NonAxis = -1,
    TX = 0, TY = 1, TZ = 2,

    Yaw = 3, Pitch = 4, Roll = 5,
    Axis_MIN = TX, Axis_MAX = 5,

    Axis_COUNT = 6,
};


struct accela
{
    accela(settings_accela * _s);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> filter(Eigen::Vector3d eul, Eigen::Vector3d T, double dt);
    void center() { first_run = true; }
private:
    settings_accela * s = nullptr;
    double last_output[6] {}, deltas[6] {};
    bool first_run = true;
};
