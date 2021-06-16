import {
  DispenserStateRowsType,
  DoorStateRowsType,
  FleetStateRowsType,
  HealthRowsType,
  IngestorStateRowsType,
  LiftStateRowsType,
  TaskSummaryRowsType,
} from '../../lib';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001').toISOString();

export const getDispenserLogs = (): DispenserStateRowsType => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      payload: 'Test' + i,
      state: 'OPEN',
      guid: 'dispenser_test',
      created: timestamp,
    });
  }
  return rows;
};

export const getDoorLogs = (): DoorStateRowsType => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      created: timestamp,
      payload: 'Test' + i,
      state: 'OPEN',
      name: 'door_test',
    });
  }
  return rows;
};

export const getFleetLogs = (): FleetStateRowsType => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      created: timestamp,
      payload: 'Test' + i,
      name: 'fleet_test',
      fleet_name: 'test',
      robots: 'test',
      robot_battery_percent: 'test',
      robot_location: 'test',
      robot_mode: 'test',
      robot_model: 'test',
      robot_name: 'test',
      robot_seq: 1,
      robot_task_id: 'test',
    });
  }
  return rows as FleetStateRowsType;
};

export const getHealthLogs = (): HealthRowsType => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      device: 'door',
      actor_id: 'door-1',
      health_status: 'DEAD',
      health_message: 'this is a message',
      created: timestamp,
      payload: 'Test' + i,
    });
  }
  return rows;
};

export const getIngestorLogs = (): IngestorStateRowsType =>
  getDispenserLogs() as IngestorStateRowsType;

export const getLiftLogs = (): LiftStateRowsType => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      state: 'AVG',
      door_state: 'Closed',
      destination_floor: 'L1',
      motion_state: 'DOWN',
      current_floor: 'L2',
      session_id: 'session',
      created: timestamp,
      payload: 'Test' + i,
    });
  }
  return rows;
};

export const getTaskSummaryLogs = (): TaskSummaryRowsType => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      created: timestamp,
      payload: 'Test' + i,
      fleet_name: 'Test',
      task_id: i.toString(),
      task_profile: { test: 'test' },
      state: 'test',
      status: 'test',
      submission_time: 'test',
      start_time: 'test',
      end_time: 'test',
      robot_name: 'test',
    });
  }
  return rows;
};
