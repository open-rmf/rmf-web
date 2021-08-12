-- upgrade --
CREATE TABLE IF NOT EXISTS "authevents" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "username" VARCHAR(100),
    "user_keycloak_id" VARCHAR(100),
    "event_type" VARCHAR(100) NOT NULL,
    "realm_id" VARCHAR(100),
    "client_id" VARCHAR(100),
    "ip_address" VARCHAR(50),
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP
);
CREATE TABLE IF NOT EXISTS "container" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "name" TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS "device" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "type" TEXT NOT NULL,
    "actor" VARCHAR(25)
);
CREATE TABLE IF NOT EXISTS "dispenserstate" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "state" VARCHAR(7) NOT NULL,
    "guid" VARCHAR(200) NOT NULL,
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP
);
COMMENT ON COLUMN "dispenserstate"."state" IS 'IDLE: idle\nBUSY: busy\nOFFLINE: offline';
CREATE TABLE IF NOT EXISTS "door" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "name" VARCHAR(100) NOT NULL
);
CREATE TABLE IF NOT EXISTS "doorstate" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "state" VARCHAR(7) NOT NULL  DEFAULT 'unknown',
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP,
    "door_id" INT REFERENCES "door" ("id") ON DELETE CASCADE
);
COMMENT ON COLUMN "doorstate"."state" IS 'CLOSED: closed\nMOVING: moving\nOPEN: open\nOFFLINE: offline\nUNKNOWN: unknown';
CREATE TABLE IF NOT EXISTS "fleet" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "name" VARCHAR(100) NOT NULL
);
CREATE TABLE IF NOT EXISTS "healthstatus" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "health_status" VARCHAR(25),
    "health_message" TEXT,
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP,
    "device_id" INT REFERENCES "device" ("id") ON DELETE CASCADE
);
CREATE TABLE IF NOT EXISTS "ingestorstate" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "state" VARCHAR(7) NOT NULL  DEFAULT 'offline',
    "guid" VARCHAR(200) NOT NULL,
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP
);
COMMENT ON COLUMN "ingestorstate"."state" IS 'IDLE: idle\nBUSY: busy\nOFFLINE: offline';
CREATE TABLE IF NOT EXISTS "lift" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "name" VARCHAR(100) NOT NULL
);
CREATE TABLE IF NOT EXISTS "liftstate" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "door_state" VARCHAR(6) NOT NULL  DEFAULT 'closed',
    "state" VARCHAR(9) NOT NULL  DEFAULT 'unknown',
    "destination_floor" VARCHAR(20) NOT NULL,
    "motion_state" VARCHAR(7) NOT NULL  DEFAULT 'stopped',
    "current_floor" VARCHAR(20) NOT NULL,
    "session_id" VARCHAR(200) NOT NULL,
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP,
    "lift_id" INT NOT NULL REFERENCES "lift" ("id") ON DELETE CASCADE
);
COMMENT ON COLUMN "liftstate"."door_state" IS 'DOOR_CLOSED: closed\nDOOR_MOVING: moving\nDOOR_OPEN: open';
COMMENT ON COLUMN "liftstate"."state" IS 'MODE_AGV: avg\nMODE_EMERGENCY: emergency\nMODE_FIRE: fire\nMODE_HUMAN: human\nMODE_OFFLINE: offline\nMODE_UNKNOWN: unknown';
COMMENT ON COLUMN "liftstate"."motion_state" IS 'MOTION_DOWN: down\nMOTION_STOPPED: stopped\nMOTION_UNKNOWN: unknown\nMOTION_UP: up';
CREATE TABLE IF NOT EXISTS "rawlog" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "level" VARCHAR(8) NOT NULL  DEFAULT 'info',
    "message" TEXT NOT NULL,
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP,
    "container_id" INT REFERENCES "container" ("id") ON DELETE CASCADE
);
COMMENT ON COLUMN "rawlog"."level" IS 'CRITICAL: critical\nERROR: error\nWARN: warn\nINFO: info\nDEBUG: debug\nUNKNOWN: unknown';
CREATE TABLE IF NOT EXISTS "robot" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "name" VARCHAR(100) NOT NULL,
    "model" VARCHAR(100)
);
CREATE TABLE IF NOT EXISTS "fleetstate" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP,
    "robot_battery_percent" VARCHAR(200) NOT NULL,
    "robot_location" VARCHAR(200) NOT NULL,
    "robot_mode" VARCHAR(13) NOT NULL  DEFAULT 'closed',
    "robot_seq" INT NOT NULL,
    "robot_task_id" VARCHAR(200) NOT NULL,
    "fleet_id" INT REFERENCES "fleet" ("id") ON DELETE CASCADE,
    "robot_id" INT REFERENCES "robot" ("id") ON DELETE CASCADE
);
COMMENT ON COLUMN "fleetstate"."robot_mode" IS 'MODE_IDLE: closed\nMODE_CHARGING: charging\nMODE_MOVING: moving\nMODE_PAUSED: paused\nMODE_WAITING: waiting\nMODE_EMERGENCY: emergency\nMODE_GOING_HOME: going_home\nMODE_DOCKING: docking\nMODE_ADAPTER_ERROR: adapter_error';
CREATE TABLE IF NOT EXISTS "tasksummary" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "created" TIMESTAMPTZ NOT NULL  DEFAULT CURRENT_TIMESTAMP,
    "task_id" VARCHAR(50) NOT NULL,
    "task_profile" JSONB NOT NULL,
    "state" VARCHAR(9) NOT NULL  DEFAULT 'pending',
    "status" VARCHAR(50),
    "submission_time" JSONB NOT NULL,
    "start_time" JSONB NOT NULL,
    "end_time" JSONB NOT NULL,
    "fleet_id" INT REFERENCES "fleet" ("id") ON DELETE CASCADE,
    "robot_id" INT REFERENCES "robot" ("id") ON DELETE CASCADE
);
COMMENT ON COLUMN "tasksummary"."state" IS 'STATE_ACTIVE: active\nSTATE_CANCELLED: cancelled\nSTATE_COMPLETED: completed\nSTATE_FAILED: failed\nSTATE_PENDING: pending\nSTATE_QUEUED: queued';
CREATE TABLE IF NOT EXISTS "aerich" (
    "id" SERIAL NOT NULL PRIMARY KEY,
    "version" VARCHAR(255) NOT NULL,
    "app" VARCHAR(20) NOT NULL,
    "content" JSONB NOT NULL
);
