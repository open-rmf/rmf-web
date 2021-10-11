import {
  DispenserStateRowsType,
  DoorStateRowsType,
  FleetStateRowsType,
  HealthRowsType,
  IngestorStateRowsType,
  LiftStateRowsType,
  TaskSummaryRowsType,
} from '.';
import { ReportConfigProps } from './utils';

const timestamp = new Date('Mon Jan  1 00:00:02 UTC 2001').toISOString();

export const getDispenserLogs = (): DispenserStateRowsType => {
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      state: 'OPEN',
      guid: `dispenser_test_${i}`,
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
      state: 'OPEN',
      door: { id: i, name: 'door_test' },
    });
  }
  return rows;
};

export const getFleetLogs = (): FleetStateRowsType => {
  const rows: FleetStateRowsType = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      created: timestamp,
      fleet: { id: 1, name: 'fleet_test' },
      robot: { id: i, name: 'robot_test', model: 'model' },
      robot_battery_percent: 'test',
      robot_location: 'test',
      robot_mode: 'test',
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
      device: { id: i, type: 'door', actor: 'door-1' },
      health_status: 'DEAD',
      health_message: 'this is a message',
      created: timestamp,
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
      lift: { id: i, name: `lift_${i}` },
    });
  }
  return rows;
};

export const getTaskSummaryLogs = (): TaskSummaryRowsType => {
  const exampleData = {
    task_id: 'test',
    submission_time: { sec: 131, nanosec: 553000000 },
    description: {
      start_time: { sec: 1623383402, nanosec: 0 },
      priority: { value: 0 },
      task_type: { type: 1 },
      station: { task_id: '', robot_type: '', place_name: '' },
      loop: { task_id: '', robot_type: '', num_loops: 1, start_name: '', finish_name: '' },
      delivery: {
        task_id: 1,
        items: [],
        pickup_place_name: '',
        pickup_dispenser: '',
        pickup_behavior: { name: '', parameters: [] },
        dropoff_place_name: '',
        dropoff_ingestor: '',
        dropoff_behavior: { name: '', parameters: [] },
      },
      clean: { start_waypoint: '' },
    },
  };
  const rows = [];
  for (let i = 0; i < 200; i++) {
    rows.push({
      created: timestamp,
      fleet: { id: 1, name: 'fleet_test' },
      robot: { id: i, name: 'robot_test', model: 'model' },
      task_id: i.toString(),
      task_profile: exampleData,
      state: 'test',
      status: 'test',
      submission_time: { sec: 131, nanosec: 553000000 },
      start_time: { sec: 131, nanosec: 553000000 },
      end_time: { sec: 131, nanosec: 553000000 },
    });
  }
  return rows;
};

export const reportConfigProps: ReportConfigProps = {
  toLogDate: new Date(),
  fromLogDate: new Date(),
  onSelectFromDate: (/* date: Date */) => {
    /* no-op */
  },
  onSelectToDate: (/* date: Date */) => {
    /* no-op */
  },
};
