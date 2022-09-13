import type { RobotState } from 'api-client';

export function makeRobot(robotState?: Partial<RobotState>): Required<RobotState> {
  return {
    name: 'test',
    battery: 1,
    location: { map: 'test_level', x: 0, y: 0, yaw: 0 },
    status: 'idle',
    task_id: 'test_task_id',
    issues: [
      JSON.parse(`{
        "category": "object category",
        "detail": {
          "first_field": "first_value",
          "second_field": "second_value",
          "third_field": "third_value"
        }
      }`),
      JSON.parse(`{
        "category": "array category",
        "detail": [
          "first_item",
          "second_item",
          "third_item",
        ]
      }`),
      JSON.parse(`{
        "category": "string category",
        "detail": "string value"
      }`),
    ],
    unix_millis_time: 0,
    ...robotState,
  };
}
