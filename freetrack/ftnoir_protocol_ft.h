/* Copyright (c) 2013-2015 Stanislaw Halik <sthalik@misaki.pl>
 * Copyright (c) 2015 Wim Vriend
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */

#pragma once


#include <QProcess>
#include <QString>
#include <QMutex>

#include <QDebug>

#include <cinttypes>
#include "freetrackclient/fttypes.h"

#include "shm.h"

#include <memory>
#include <fagx_datatype.h>


class freetrack : QObject
{
    Q_OBJECT

public:
    freetrack() = default;
    ~freetrack();
    bool initialize();
    void pose(const double* pose, const double*);
    QString game_name();
public slots:
    void on_pose6d_data(double t, Pose6DoF pose);
private:
    shm_wrapper shm { FREETRACK_HEAP, FREETRACK_MUTEX, sizeof(FTHeap) };
    FTHeap* pMemData { (FTHeap*) shm.ptr() };

    QProcess dummyTrackIR;

    int intGameID = -1;
    QString connected_game;
    QMutex game_name_mutex;

    void start_dummy();

public:
    static void set_protocols(bool ft, bool npclient);
};
