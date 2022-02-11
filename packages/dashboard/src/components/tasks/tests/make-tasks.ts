// import {
//   Clean,
//   Delivery,
//   Loop,
//   Station,
//   Task,
//   TaskDescription,
//   TaskProfile,
//   TaskSummary,
// } from 'api-client';
// import { TaskSummary as RmfTaskSummary } from 'rmf-models';
import {
  TaskEventLog,
  TaskState,
  Phases,
  LogEntry,
  Tier,
  Booking,
  Detail,
  Status,
  Phase,
  EventState,
} from 'api-client';
import react from 'react';
/**
 * FIXME: These `makeX` functions are duplicated in `react-components`.
 * Whats the best way to dedupe this?
 *   1. Make a package for testing utils.
 *   2. Put these in `api-client`. We can also consider generating `makeX` functions for all models.
 *     1. `makeX` functions should only be generated for models derived from ROS.
 *     2. How do we know which models are from ROS?
 *   3. export these from `react-components`.
 */
// const baseCleanDesc: Clean = {
//   start_waypoint: '',
// };

// export function makeClean(clean: Partial<Clean> = {}): Clean {
//   return { ...baseCleanDesc, ...clean };
// }

// const baseDeliveryDesc: Delivery = {
//   task_id: '',
//   pickup_behavior: {
//     name: '',
//     parameters: [],
//   },
//   pickup_dispenser: '',
//   pickup_place_name: '',
//   dropoff_behavior: {
//     name: '',
//     parameters: [],
//   },
//   dropoff_ingestor: '',
//   dropoff_place_name: '',
//   items: [],
// };

// export function makeDelivery(delivery: Partial<Delivery> = {}): Delivery {
//   return { ...baseDeliveryDesc, ...delivery };
// }

// const baseLoopDesc: Loop = {
//   task_id: '',
//   start_name: '',
//   finish_name: '',
//   num_loops: 0,
//   robot_type: '',
// };

// export function makeLoop(loop: Partial<Loop> = {}): Loop {
//   return { ...baseLoopDesc, ...loop };
// }

// const baseStationDesc: Station = {
//   task_id: '',
//   place_name: '',
//   robot_type: '',
// };

// export function makeStation(station: Partial<Station> = {}): Station {
//   return { ...baseStationDesc, ...station };
// }

// const baseTaskDescription: TaskDescription = {
//   clean: baseCleanDesc,
//   delivery: baseDeliveryDesc,
//   loop: baseLoopDesc,
//   station: baseStationDesc,
//   task_type: { type: 0 },
//   priority: { value: 0 },
//   start_time: { sec: 0, nanosec: 0 },
// };

// export function makeTaskDescription(
//   taskDescription: Partial<TaskDescription> = {},
// ): TaskDescription {
//   return { ...baseTaskDescription, ...taskDescription };
// }

// const baseTaskProfile: TaskProfile = {
//   task_id: '',
//   submission_time: { sec: 0, nanosec: 0 },
//   description: baseTaskDescription,
// };

// export function makeTaskProfile(taskProfile: Partial<TaskProfile> = {}): TaskProfile {
//   return {
//     ...baseTaskProfile,
//     ...taskProfile,
//   };
// }

// const baseTaskSummary: TaskSummary = {
//   task_id: '',
//   fleet_name: '',
//   robot_name: '',
//   task_profile: baseTaskProfile,
//   end_time: { sec: 0, nanosec: 0 },
//   start_time: { sec: 0, nanosec: 0 },
//   state: 1,
//   status: '',
//   submission_time: { sec: 0, nanosec: 0 },
// };

// export function makeTaskSummary(taskSummary: Partial<TaskSummary> = {}): TaskSummary {
//   return {
//     ...baseTaskSummary,
//     ...taskSummary,
//   };
// }

// export function makeTaskSummaryWithPhases(
//   id: string,
//   numberOfPhases: number,
//   currentPhase: number,
// ): TaskSummary {
//   let status = '';
//   for (let i = 0; i < numberOfPhases; i++) {
//     if (currentPhase === i + 1) {
//       status += '*';
//     }
//     status += `Phase ${i + 1}\ntest phase ${i + 1}\n\n`;
//   }
//   status = status.trimEnd();
//   return makeTaskSummary({
//     task_id: id,
//     state: RmfTaskSummary.STATE_ACTIVE,
//     status: status,
//     fleet_name: 'test_fleet',
//     robot_name: 'test_robot',
//   });
// }

// export function makeTaskWithPhases(
//   taskId: string,
//   numberOfPhases: number,
//   currentPhase: number,
// ): Task {
//   const taskSummary = makeTaskSummaryWithPhases(taskId, numberOfPhases, currentPhase);
//   return {
//     task_id: taskId,
//     authz_grp: 'test_group',
//     progress: { status: '' },
//     summary: taskSummary,
//   };
// }

export function makeLogEntry(): LogEntry {
  return {
    seq: 0,
    tier: Tier.Info,
    unix_millis_time: 90000,
    text: 'Open Door',
  };
}

export function makePhases(logEntries: Array<LogEntry>): Phases {
  return {
    log: logEntries,
    events: { '0': logEntries },
  };
}

export function makeEventState(eventId: number, deps?: Array<number>): EventState {
  return {
    id: eventId,
    status: Status.Underway,
    name: '-',
    detail: '',
    deps: deps,
  };
}

export function makePhase(phaseId: number): Phase {
  const eventState0 = makeEventState(0);
  const eventState1 = makeEventState(1, [0]);
  return {
    id: phaseId,
    category: '',
    detail: '',
    unix_millis_start_time: 90000,
    unix_millis_finish_time: 90009,
    original_estimate_millis: 900000,
    estimate_millis: 1000,
    events: { '0': eventState0, '1': eventState1 },
  };
}

export function makeTaskEventLog(taskId: string): TaskEventLog {
  const logEntry = makeLogEntry();
  const phases = makePhases([logEntry]);
  return {
    task_id: taskId,
    log: [logEntry],
    phases: { '0': phases },
  };
}

export function makeBooking(bookingId: string): Booking {
  return {
    id: bookingId,
    unix_millis_earliest_start_time: 90000,
    priority: '1',
    labels: ['', ''],
  };
}

export function makeTaskEventState(taskId: string): TaskState {
  const booking = makeBooking(taskId);
  const phase = makePhase(0);
  return {
    booking: booking,
    category: '',
    detail: '',
    unix_millis_start_time: 90000,
    unix_millis_finish_time: 90009,
    original_estimate_millis: 90009,
    estimate_millis: 1000,
    status: Status.Underway,
    completed: [],
    phases: { '0': phase },
  };
}
