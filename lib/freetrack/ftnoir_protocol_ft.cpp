/* Copyright (c) 2013-2015, 2017 Stanislaw Halik <sthalik@misaki.pl>
 * Copyright (c) 2015 Wim Vriend
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 */


#include "ftnoir_protocol_ft.h"
#include "freetrack/csv/csv.h"
#include "FlightAgxSettings.h"

#include <cstddef>
#define _USE_MATH_DEFINES
#include <cmath>
#include <windows.h>
#include <math.h>

freetrack::~freetrack()
{
    dummyTrackIR.close();
}

static_assert(sizeof(LONG) == sizeof(std::int32_t));
static_assert(sizeof(LONG) == 4u);

#define TX 0
#define TY 1
#define TZ 2
#define YAW 3
#define PITCH 4
#define ROLL 5


never_inline void store(float volatile& place, const float value)
{
    union
    {
        float f32;
        LONG i32;
    } value_ {};

    value_.f32 = value;

    static_assert(sizeof(value_) == sizeof(float));
    static_assert(offsetof(decltype(value_), f32) == offsetof(decltype(value_), i32));

    (void)InterlockedExchange((LONG volatile*)&place, value_.i32);
}

template<typename t>
static void store(t volatile& place, t value)
{
    static_assert(sizeof(t) == 4u);
    (void)InterlockedExchange((LONG volatile*) &place, (LONG)value);
}

static std::int32_t load(std::int32_t volatile& place)
{
    return InterlockedCompareExchange((volatile LONG*) &place, 0, 0);
}


void freetrack::on_pose6d_data(double t, Pose6DoF pose) {
    double data[6] = {0};
    data[0] = - pose.second.x()*100;
    data[1] = pose.second.y()*100;
    data[2] = - pose.second.z()*100;
    auto eul = pose.first;
    data[3] = eul.x();
    data[4] = eul.y();
    data[5] = eul.z();

    this->pose(data, data);
}

void freetrack::pose(const double* headpose, const double* raw)
{
    constexpr double d2r = M_PI/180.0;

    const float yaw = float(-headpose[YAW] * d2r);
    const float roll = float(headpose[ROLL] * d2r);
    const float tx = float(headpose[TX] * 10);
    const float ty = float(headpose[TY] * 10);
    const float tz = float(headpose[TZ] * 10);

    // HACK: Falcon BMS makes a "bump" if pitch is over the value -sh 20170615
    const bool is_crossing_90 = std::fabs(headpose[PITCH] - 90) < .15;
    const float pitch = float(-d2r * (is_crossing_90 ? 89.86 : headpose[PITCH]));

    FTHeap* const ft = pMemData;
    FTData* const data = &ft->data;

    store(data->X, tx);
    store(data->Y, ty);
    store(data->Z, tz);

    store(data->Yaw, yaw);
    store(data->Pitch, pitch);
    store(data->Roll, roll);

    store(data->RawYaw, float(-raw[YAW] * d2r));
    store(data->RawPitch, float(raw[PITCH] * d2r));
    store(data->RawRoll, float(raw[ROLL] * d2r));
    store(data->RawX, float(raw[TX] * 10));
    store(data->RawY, float(raw[TY] * 10));
    store(data->RawZ, float(raw[TZ] * 10));

    const std::int32_t id = load(ft->GameID);

    if (intGameID != id)
    {
        QString gamename;
        union  {
            unsigned char table[8];
            std::int32_t ints[2];
        } t {};

        t.ints[0] = 0; t.ints[1] = 0;

        (void)CSV::getGameData(id, t.table, settings->support_games_csv.c_str(), gamename);

        {
            // FTHeap pMemData happens to be aligned on a page boundary by virtue of
            // memory mapping usage (MS Windows equivalent of mmap(2)).
            static_assert((offsetof(FTHeap, table) & (sizeof(LONG)-1)) == 0);

            for (unsigned k = 0; k < 2; k++)
                store(pMemData->table_ints[k], t.ints[k]);
        }

        store(ft->GameID2, id);
        store(data->DataID, 0u);

        intGameID = id;

        if (gamename.isEmpty())
            gamename = tr("Unknown game");

        QMutexLocker foo(&game_name_mutex);
        connected_game = gamename;
    }
    else
        (void)InterlockedAdd((LONG volatile*)&data->DataID, 1);
}

QString freetrack::game_name()
{
    QMutexLocker foo(&game_name_mutex);
    return connected_game;
}

void freetrack::start_dummy() {
    static const QString program(settings->trackir_path.c_str());
    dummyTrackIR.setProgram("\"" + program + "\"");
    dummyTrackIR.start();
}

void freetrack::set_protocols(bool ft, bool npclient)
{
    /*
    QSettings settings_ft("Freetrack", "FreetrackClient");
    QSettings settings_npclient("NaturalPoint", "NATURALPOINT\\NPClient Location");

    if (ft)
        settings_ft.setValue("Path", program_dir);
    else
        settings_ft.setValue("Path", "");

    if (npclient)
        settings_npclient.setValue("Path", program_dir);
    else
        settings_npclient.setValue("Path", "");*/
}

bool freetrack::initialize()
{
    if (!shm.success())
    {
        qDebug() << "Freetrack start failed";
        return false;
    }

    bool use_ft = settings->use_ft;
    bool use_npclient = settings->use_npclient;

    set_protocols(use_ft, use_npclient);

    pMemData->data.DataID = 1;
    pMemData->data.CamWidth = 100;
    pMemData->data.CamHeight = 250;

#if 0
    store(pMemData->data.X1, float(100));
    store(pMemData->data.Y1, float(200));
    store(pMemData->data.X2, float(300));
    store(pMemData->data.Y2, float(200));
    store(pMemData->data.X3, float(300));
    store(pMemData->data.Y3, float(100));
#endif

    store(pMemData->GameID2, 0);

    for (unsigned k = 0; k < 2; k++)
        store(pMemData->table_ints[k], 0);

    // more games need the dummy executable than previously thought
    if (use_npclient)
        start_dummy();

    return true;
}
