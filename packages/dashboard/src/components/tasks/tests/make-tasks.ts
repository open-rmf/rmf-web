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
  //Phases,
  //LogEntry,
  //Tier,
  //Booking,
  //Detail,
  //Status,
  //Phase,
  //EventState,
} from 'api-client';
//import react from 'react';
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

// export function makeLogEntry(): LogEntry {
//   return {
//     seq: 0,
//     tier: Tier.Info,
//     unix_millis_time: 90000,
//     text: 'Open Door',
//   };
// }

// export function makePhases(logEntries: Array<LogEntry>): Phases {
//   return {
//     log: logEntries,
//     events: { '0': logEntries },
//   };
// }

// export function makeEventState(eventId: number, deps?: Array<number>): EventState {
//   return {
//     id: eventId,
//     status: Status.Underway,
//     name: '-',
//     detail: '',
//     deps: deps,
//   };
// }

// export function makePhase(phaseId: number): Phase {
//   const eventState0 = makeEventState(0);
//   const eventState1 = makeEventState(1, [0]);
//   return {
//     id: phaseId,
//     category: '',
//     detail: '',
//     unix_millis_start_time: 90000,
//     unix_millis_finish_time: 90009,
//     original_estimate_millis: 900000,
//     estimate_millis: 1000,
//     events: { '0': eventState0, '1': eventState1 },
//   };
// }

// export function makeTaskEventLog(taskId: string): TaskEventLog {
//   const logEntry = makeLogEntry();
//   const phases = makePhases([logEntry]);
//   return {
//     task_id: taskId,
//     log: [logEntry],
//     phases: { '0': phases },
//   };
// }

// export function makeBooking(bookingId: string): Booking {
//   return {
//     id: bookingId,
//     unix_millis_earliest_start_time: 90000,
//     priority: '1',
//     labels: ['', ''],
//   };
// }

// export function makeTaskEventState(taskId: string): TaskState {
//   const booking = makeBooking(taskId);
//   const phase = makePhase(0);
//   return {
//     booking: booking,
//     category: '',
//     detail: '',
//     unix_millis_start_time: 90000,
//     unix_millis_finish_time: 90009,
//     original_estimate_millis: 90009,
//     estimate_millis: 1000,
//     status: Status.Underway,
//     completed: [],
//     phases: { '0': phase },
//   };
// }

export function makeTaskState(taskId: string): TaskState {
  const state = JSON.parse(`{
      "booking": {
        "id": "delivery_2021:11:08:23:50",
        "unix_millis_earliest_start_time": 1636388400000,
        "priority": "none",
        "automatic": false
      },
      "category": "Multi-Delivery",
      "detail": [
        {
          "category": "Pick Up",
          "params": {
            "location": "Kitchen",
            "items": [
              {
                "type": "soda",
                "quantity": 1
              },
              {
                "type": "water",
                "quantity": 1
              }
            ]
          }
        },
        {
          "category": "Drop Off",
          "params": {
            "location": "room_203",
            "items": [
              {
                "type": "soda",
                "quantity": 1
              }
            ]
          }
        },
        {
          "category": "Drop Off",
          "params": {
            "location": "room_521",
            "items": [
              {
                "type": "water",
                "quantity": 1
              }
            ]
          }
        }
      ],
      "unix_millis_start_time": 1636388410000,
      "estimate_millis": 2000000,
      "phases": {
        "1": {
          "id": 1,
          "category": "Pick Up",
          "detail": {
            "location": "Kitchen",
            "items": [
              {
                "type": "soda",
                "quantity": 1
              },
              {
                "type": "water",
                "quantity": 1
              }
            ]
          },
          "estimate_millis": 600000,
          "final_event_id": 0,
          "events": {
            "0": {
              "id": 0,
              "status": "completed",
              "name": "Pick Up Sequence",
              "detail": "",
              "deps": [1, 2]
            },
            "1": {
              "id": 1,
              "status": "completed",
              "name": "Go to [place:kitchen]",
              "detail": "",
              "deps": [3, 4, 8]
            },
            "2": {
              "id": 2,
              "status": "completed",
              "name": "Receive items",
              "detail": [
                {
                  "type": "soda",
                  "quantity": 1
                },
                {
                  "type": "water",
                  "quantity": 1
                }
              ],
              "deps": []
            },
            "3": {
              "id": 3,
              "status": "completed",
              "name": "Move to [place:kitchen_door_exterior]",
              "detail": "",
              "deps": []
            },
            "4": {
              "id": 4,
              "status": "completed",
              "name": "Pass through [door:kitchen_door]",
              "detail": "",
              "deps": [5, 6, 7]
            },
            "5": {
              "id": 5,
              "status": "completed",
              "name": "Wait for [door:kitchen_door] to open",
              "detail": "",
              "deps": []
            },
            "6": {
              "id": 6,
              "status": "completed",
              "name": "Move to [place:kitchen_door_interior]",
              "detail": "",
              "deps": []
            },
            "7": {
              "id": 7,
              "status": "completed",
              "name": "Wait for [door:kitchen_door] to close",
              "detail": "",
              "deps": []
            },
            "8": {
              "id": 8,
              "status": "completed",
              "name": "Move to [place:kitchen]",
              "detail": "",
              "deps": []
            }
          }
        },
        "2": {
          "id": 2,
          "category": "Drop Off",
          "detail": {
            "location": "room_203",
            "items": [
              {
                "type": "soda",
                "quantity": 1
              }
            ]
          },
          "estimate_millis": 720000,
          "final_event_id": 0,
          "events": {
            "0": {
              "id": 0,
              "status": "underway",
              "name": "Drop Off Sequence",
              "detail": "",
              "deps": [1, 2]
            },
            "1": {
              "id": 1,
              "status": "underway",
              "name": "Go to [place:room_203]",
              "detail": "",
              "deps": [3, 4, 8, 9, 14]
            },
            "2": {
              "id": 2,
              "status": "standby",
              "name": "Unload items",
              "detail": [
                {
                  "type": "soda",
                  "quantity": 1
                }
              ],
              "deps": []
            },
            "3": {
              "id": 3,
              "status": "completed",
              "name": "Move to [place:kitchen_door_interior]",
              "detail": "",
              "deps": []
            },
            "4": {
              "id": 4,
              "status": "underway",
              "name": "Pass through [door:kitchen_door]",
              "detail": "",
              "deps": [5, 6, 7]
            },
            "5": {
              "id": 5,
              "status": "underway",
              "name": "Wait for [door:kitchen_door] to open",
              "detail": "",
              "deps": []
            },
            "6": {
              "id": 6,
              "status": "standby",
              "name": "Move to [place:kitchen_door_exterior]",
              "detail": "",
              "deps": []
            },
            "7": {
              "id": 7,
              "status": "standby",
              "name": "Wait for [door:kitchen_door] to close",
              "detail": "",
              "deps": []
            },
            "8": {
              "id": 8,
              "status": "standby",
              "name": "Move to [place:lift_lobby_05_floor_B1]",
              "detail": "",
              "deps": []
            },
            "9": {
              "id": 9,
              "status": "standby",
              "name": "Take [lift:lift_05_03] to [place:lift_lobby_05_floor_L2]",
              "detail": "",
              "deps": [10, 11, 12, 13]
            },
            "10": {
              "id": 10,
              "status": "underway",
              "name": "Wait for lift",
              "detail": "Currently assigned [lift:lift_05_03]",
              "deps": []
            },
            "11": {
              "id": 11,
              "status": "standby",
              "name": "Move to [place:lift_05_03_floor_B1]",
              "detail": "",
              "deps": []
            },
            "12": {
              "id": 12,
              "status": "standby",
              "name": "Lift [lift:lift_05_03] to [place:lift_05_03_floor_2]",
              "detail": "",
              "deps": []
            },
            "13": {
              "id": 13,
              "status": "standby",
              "name": "Wait for [lift:lift_05_03] to open",
              "detail": "",
              "deps": []
            },
            "14": {
              "id": 14,
              "status": "standby",
              "name": "Move to [place:room_203]",
              "detail": "",
              "deps": []
            }
          }
        },
        "3": {
          "id": 3,
          "category": "Drop Off",
          "detail": {
            "location": "room_521",
            "items": [
              {
                "type": "water",
                "quantity": 1
              }
            ]
          },
          "estimate_millis": 680000
        }
      },
      "completed": [ 1 ],
      "active": 2,
      "pending": [ 3 ]
    }
    `);
  state.booking.id = taskId;
  return state;
}

export function makeTaskLog(taskId: string): TaskEventLog {
  const log = JSON.parse(`{
      "task_id": "delivery_2021:11:08:23:50",
      "log": [
        {
          "seq": 0,
          "tier": "info",
          "unix_millis_time": 1636388410000,
          "text": "Beginning task"
        }
      ],
      "phases": {
        "1": {
          "log": [
            {
              "seq": 0,
              "tier": "info",
              "unix_millis_time": 1636388410000,
              "text": "Beginning phase"
            }
          ],
          "events": {
            "1": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388409995,
                "text": "Generating plan to get from [place:parking_03] to [place:kitchen]"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388409996,
                "text": "Finished generating plan to get from [place:parking_03] to [place:kitchen]"
              },
              {
                "seq": 2,
                "tier": "info",
                "unix_millis_time": 1636388414500,
                "text": "Finished: Move to [place:kitchen_door_exterior]"
              },
              {
                "seq": 3,
                "tier": "info",
                "unix_millis_time": 1636388415000,
                "text": "Finished: Wait for [door:kitchen_door] to open"
              },
              {
                "seq": 4,
                "tier": "info",
                "unix_millis_time": 1636388418001,
                "text": "Finished: Move to [place:kitchen_door_interior]"
              },
              {
                "seq": 5,
                "tier": "info",
                "unix_millis_time": 1636388419111,
                "text": "Finished: Wait for [door:kitchen_door] to close"
              },
              {
                "seq": 6,
                "tier": "info",
                "unix_millis_time": 1636388421121,
                "text": "Finished: Move to [place:kitchen]"
              }
            ],
            "2": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388421421,
                "text": "Requested [item:soda], [item:water]"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388421521,
                "text": "Request acknowledged"
              },
              {
                "seq": 2,
                "tier": "info",
                "unix_millis_time": 1636388430000,
                "text": "Received [item:soda]"
              },
              {
                "seq": 3,
                "tier": "info",
                "unix_millis_time": 1636388440000,
                "text": "Received [item:water]"
              }
            ],
            "3": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388410000,
                "text": "Moving towards [place:kitchen_door_exterior] from [place:parking_03]"
              },
              {
                "seq": 1,
                "tier": "warning",
                "unix_millis_time": 1636388411000,
                "text": "Delayed by obstacle blocking [robot:deliverbot_01]"
              },
              {
                "seq": 2,
                "tier": "info",
                "unix_millis_time": 1636388414500,
                "text": "Arrived at [place:kitchen_door_exterior]"
              }
            ],
            "5": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388414500,
                "text": "Requested [door:kitchen_door] to open"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388414600,
                "text": "[door:kitchen_door] acknowledged request"
              },
              {
                "seq": 2,
                "tier": "info",
                "unix_millis_time": 1636388415000,
                "text": "[door:kitchen_door] has opened"
              }
            ],
            "6": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388415010,
                "text": "Moving towards [place:kitchen_door_interior] from [place:kitchen_door_exterior]"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388418000,
                "text": "Arrived at [place:kitchen_door_interior]"
              }
            ],
            "7": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388418010,
                "text": "Requested [door:kitchen_door] to close"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388418110,
                "text": "[door:kitchen_door] acknowledged request"
              },
              {
                "seq": 2,
                "tier": "info",
                "unix_millis_time": 1636388419110,
                "text": "[door:kitchen] has closed"
              }
            ],
            "8": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388419120,
                "text": "Moving towards [place:kitchen] from [place:kitchen_door_interior]"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388421120,
                "text": "Arrived at [place:kitchen]"
              }
            ]
          }
        },
        "2": {
          "log": [
            {
              "seq": 0,
              "tier": "info",
              "unix_millis_time": 1636388444500,
              "text": "Beginning phase"
            }
          ],
          "events": {
            "1": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388440000,
                "text": "Generating plan to get from [place:kitchen] to [place:room_203]"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388440010,
                "text": "Finished generating plan to get from [place:kitchen_03] to [place:room_203]"
              },
              {
                "seq": 2,
                "tier": "info",
                "unix_millis_time": 1636388450000,
                "text": "Finished: Move to [place:kitchen_door_interior]"
              }
            ],
            "3": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388440010,
                "text": "Moving towards [place:kitchen_door_interior] from [place:kitchen]"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388450000,
                "text": "Arrived at [place:kitchen_door_interior]"
              }
            ],
            "5": [
              {
                "seq": 0,
                "tier": "info",
                "unix_millis_time": 1636388450010,
                "text": "Requested [door:kitchen_door] to open"
              },
              {
                "seq": 1,
                "tier": "info",
                "unix_millis_time": 1636388450110,
                "text": "[door:kitchen_door] acknowledged request"
              }
            ]
          }
        }
      }
    }
    `);
  log.task_id = taskId;
  return log;
}
