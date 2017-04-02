/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9-dev at Sun Mar 26 21:45:23 2017. */

#include "CommMsgs.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

const snd_msgs_Log_eLogLevel snd_msgs_Log_level_default = snd_msgs_Log_eLogLevel_INFO;


const pb_field_t snd_msgs_EmptyMsg_fields[1] = {
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_Point_fields[3] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, snd_msgs_Point, x, x, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, snd_msgs_Point, y, x, 0),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_Pose_fields[4] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, snd_msgs_Pose, x, x, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, snd_msgs_Pose, y, x, 0),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, snd_msgs_Pose, th, y, 0),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_Speed_fields[3] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, snd_msgs_Speed, left, left, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, snd_msgs_Speed, right, left, 0),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_Encoders_fields[3] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, snd_msgs_Encoders, left, left, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, snd_msgs_Encoders, right, left, 0),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_PidTunings_fields[4] = {
    PB_FIELD(  1, FLOAT   , REQUIRED, STATIC  , FIRST, snd_msgs_PidTunings, p, p, 0),
    PB_FIELD(  2, FLOAT   , REQUIRED, STATIC  , OTHER, snd_msgs_PidTunings, i, p, 0),
    PB_FIELD(  3, FLOAT   , REQUIRED, STATIC  , OTHER, snd_msgs_PidTunings, d, i, 0),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_Ir_fields[4] = {
    PB_FIELD(  1, BOOL    , REQUIRED, STATIC  , FIRST, snd_msgs_Ir, left, left, 0),
    PB_FIELD(  2, BOOL    , REQUIRED, STATIC  , OTHER, snd_msgs_Ir, center, left, 0),
    PB_FIELD(  3, BOOL    , REQUIRED, STATIC  , OTHER, snd_msgs_Ir, right, center, 0),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_Status_fields[7] = {
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , FIRST, snd_msgs_Status, speed, speed, &snd_msgs_Speed_fields),
    PB_FIELD(  3, BOOL    , REQUIRED, STATIC  , OTHER, snd_msgs_Status, starter, speed, 0),
    PB_FIELD(  4, BOOL    , REQUIRED, STATIC  , OTHER, snd_msgs_Status, estop, starter, 0),
    PB_FIELD(  5, UENUM   , REQUIRED, STATIC  , OTHER, snd_msgs_Status, colorSwitch, estop, 0),
    PB_FIELD(  6, MESSAGE , REQUIRED, STATIC  , OTHER, snd_msgs_Status, ir, colorSwitch, &snd_msgs_Ir_fields),
    PB_FIELD(  7, MESSAGE , REQUIRED, STATIC  , OTHER, snd_msgs_Status, encoders, ir, &snd_msgs_Encoders_fields),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_Log_fields[4] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, snd_msgs_Log, stamp, stamp, 0),
    PB_FIELD(  2, UENUM   , REQUIRED, STATIC  , OTHER, snd_msgs_Log, level, stamp, &snd_msgs_Log_level_default),
    PB_FIELD(  3, STRING  , REQUIRED, CALLBACK, OTHER, snd_msgs_Log, text, level, 0),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_SerialRequest_fields[12] = {
    PB_ONEOF_FIELD(type,   1, MESSAGE , ONEOF, STATIC  , FIRST, snd_msgs_SerialRequest, getStatus, getStatus, &snd_msgs_EmptyMsg_fields),
    PB_ONEOF_FIELD(type,   2, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, getEncoders, getEncoders, &snd_msgs_EmptyMsg_fields),
    PB_ONEOF_FIELD(type,   3, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, getPosition, getPosition, &snd_msgs_EmptyMsg_fields),
    PB_ONEOF_FIELD(type,   4, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, getMotorsSpeed, getMotorsSpeed, &snd_msgs_EmptyMsg_fields),
    PB_ONEOF_FIELD(type,   5, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, getPidSpeedLeft, getPidSpeedLeft, &snd_msgs_EmptyMsg_fields),
    PB_ONEOF_FIELD(type,   6, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, getPidSpeedRight, getPidSpeedRight, &snd_msgs_EmptyMsg_fields),
    PB_ONEOF_FIELD(type,   7, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, getStarterState, getStarterState, &snd_msgs_EmptyMsg_fields),
    PB_ONEOF_FIELD(type,  10, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, setMotorsSpeed, setMotorsSpeed, &snd_msgs_Speed_fields),
    PB_ONEOF_FIELD(type,  11, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, setPidSpeedLeft, setPidSpeedLeft, &snd_msgs_PidTunings_fields),
    PB_ONEOF_FIELD(type,  12, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, setPidSpeedRight, setPidSpeedRight, &snd_msgs_PidTunings_fields),
    PB_ONEOF_FIELD(type,  13, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialRequest, setEncoders, setEncoders, &snd_msgs_Encoders_fields),
    PB_LAST_FIELD
};

const pb_field_t snd_msgs_SerialResponse_fields[9] = {
    PB_ONEOF_FIELD(type,   1, MESSAGE , ONEOF, STATIC  , FIRST, snd_msgs_SerialResponse, log, log, &snd_msgs_Log_fields),
    PB_ONEOF_FIELD(type,   2, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialResponse, encoders, encoders, &snd_msgs_Encoders_fields),
    PB_ONEOF_FIELD(type,   3, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialResponse, pose, pose, &snd_msgs_Pose_fields),
    PB_ONEOF_FIELD(type,   4, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialResponse, speed, speed, &snd_msgs_Speed_fields),
    PB_ONEOF_FIELD(type,   5, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialResponse, pidSpeedLeft, pidSpeedLeft, &snd_msgs_PidTunings_fields),
    PB_ONEOF_FIELD(type,   6, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialResponse, pidSpeedRight, pidSpeedRight, &snd_msgs_PidTunings_fields),
    PB_ONEOF_FIELD(type,   7, BOOL    , ONEOF, STATIC  , UNION, snd_msgs_SerialResponse, isStarterSet, isStarterSet, 0),
    PB_ONEOF_FIELD(type,   8, MESSAGE , ONEOF, STATIC  , UNION, snd_msgs_SerialResponse, status, status, &snd_msgs_Status_fields),
    PB_LAST_FIELD
};




/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(snd_msgs_Status, speed) < 65536 && pb_membersize(snd_msgs_Status, ir) < 65536 && pb_membersize(snd_msgs_Status, encoders) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.getStatus) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.getEncoders) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.getPosition) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.getMotorsSpeed) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.getPidSpeedLeft) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.getPidSpeedRight) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.getStarterState) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.setMotorsSpeed) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.setPidSpeedLeft) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.setPidSpeedRight) < 65536 && pb_membersize(snd_msgs_SerialRequest, type.setEncoders) < 65536 && pb_membersize(snd_msgs_SerialResponse, type.log) < 65536 && pb_membersize(snd_msgs_SerialResponse, type.encoders) < 65536 && pb_membersize(snd_msgs_SerialResponse, type.pose) < 65536 && pb_membersize(snd_msgs_SerialResponse, type.speed) < 65536 && pb_membersize(snd_msgs_SerialResponse, type.pidSpeedLeft) < 65536 && pb_membersize(snd_msgs_SerialResponse, type.pidSpeedRight) < 65536 && pb_membersize(snd_msgs_SerialResponse, type.status) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_snd_msgs_EmptyMsg_snd_msgs_Point_snd_msgs_Pose_snd_msgs_Speed_snd_msgs_Encoders_snd_msgs_PidTunings_snd_msgs_Ir_snd_msgs_Status_snd_msgs_Log_snd_msgs_SerialRequest_snd_msgs_SerialResponse)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(snd_msgs_Status, speed) < 256 && pb_membersize(snd_msgs_Status, ir) < 256 && pb_membersize(snd_msgs_Status, encoders) < 256 && pb_membersize(snd_msgs_SerialRequest, type.getStatus) < 256 && pb_membersize(snd_msgs_SerialRequest, type.getEncoders) < 256 && pb_membersize(snd_msgs_SerialRequest, type.getPosition) < 256 && pb_membersize(snd_msgs_SerialRequest, type.getMotorsSpeed) < 256 && pb_membersize(snd_msgs_SerialRequest, type.getPidSpeedLeft) < 256 && pb_membersize(snd_msgs_SerialRequest, type.getPidSpeedRight) < 256 && pb_membersize(snd_msgs_SerialRequest, type.getStarterState) < 256 && pb_membersize(snd_msgs_SerialRequest, type.setMotorsSpeed) < 256 && pb_membersize(snd_msgs_SerialRequest, type.setPidSpeedLeft) < 256 && pb_membersize(snd_msgs_SerialRequest, type.setPidSpeedRight) < 256 && pb_membersize(snd_msgs_SerialRequest, type.setEncoders) < 256 && pb_membersize(snd_msgs_SerialResponse, type.log) < 256 && pb_membersize(snd_msgs_SerialResponse, type.encoders) < 256 && pb_membersize(snd_msgs_SerialResponse, type.pose) < 256 && pb_membersize(snd_msgs_SerialResponse, type.speed) < 256 && pb_membersize(snd_msgs_SerialResponse, type.pidSpeedLeft) < 256 && pb_membersize(snd_msgs_SerialResponse, type.pidSpeedRight) < 256 && pb_membersize(snd_msgs_SerialResponse, type.status) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_snd_msgs_EmptyMsg_snd_msgs_Point_snd_msgs_Pose_snd_msgs_Speed_snd_msgs_Encoders_snd_msgs_PidTunings_snd_msgs_Ir_snd_msgs_Status_snd_msgs_Log_snd_msgs_SerialRequest_snd_msgs_SerialResponse)
#endif


/* @@protoc_insertion_point(eof) */
