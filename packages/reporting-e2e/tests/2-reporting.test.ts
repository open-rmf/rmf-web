import { login } from './utils';
import fetch from 'node-fetch';

describe('reporting interactions', () => {
  before(() => browser.url('/'));
  before(login);
  before(async () => {
    const rmfServerData = [
      {
        log:
          'INFO:app.BookKeeper.task_summary:{"fleet_name": "tinyRobot", "task_id": "Loop0", "task_profile": {"task_id": "Loop0", "submission_time": {"sec": 131, "nanosec": 553000000}, "description": {"start_time": {"sec": 1623383402, "nanosec": 0}, "priority": {"value": 0}, "task_type": {"type": 1}, "station": {"task_id": "", "robot_type": "", "place_name": ""}, "loop": {"task_id": "", "robot_type": "", "num_loops": 1, "start_name": "supplies", "finish_name": "coe"}, "delivery": {"task_id": "", "items": [], "pickup_place_name": "", "pickup_dispenser": "", "pickup_behavior": {"name": "", "parameters": []}, "dropoff_place_name": "", "dropoff_ingestor": "", "dropoff_behavior": {"name": "", "parameters": []}}, "clean": {"start_waypoint": ""}}}, "state": 0, "status": "test_status", "submission_time": {"sec": 0, "nanosec": 0}, "start_time": {"sec": 1623383362, "nanosec": 348338289}, "end_time": {"sec": 1623383449, "nanosec": 79154833}, "robot_name": "tinyRobot2"}',
        stream: 'stdout',
      },
      {
        log:
          'INFO:app.BookKeeper.task_summary:{"fleet_name": "tinyRobot", "task_id": "Delivery1", "task_profile": {"task_id": "Delivery1", "submission_time": {"sec": 132, "nanosec": 553000098}, "description": {"start_time": {"sec": 1623383487, "nanosec": 0}, "priority": {"value": 0}, "task_type": {"type": 2}, "station": {"task_id": "", "robot_type": "", "place_name": ""}, "loop": {"task_id": "", "robot_type": "", "num_loops": "", "start_name": "", "finish_name": ""}, "delivery": {"task_id": "", "items": ["item1", "item2"], "pickup_place_name": "pantry", "pickup_dispenser": "coke_dispenser", "pickup_behavior": {"name": "", "parameters": []}, "dropoff_place_name": "lounge", "dropoff_ingestor": "ingestor", "dropoff_behavior": {"name": "", "parameters": []}}, "clean": {"start_waypoint": ""}}}, "state": 0, "status": "test_status2", "submission_time": {"sec": 0, "nanosec": 0}, "start_time": {"sec": 1623383362, "nanosec": 348338289}, "end_time": {"sec": 1623383449, "nanosec": 79154833}, "robot_name": "tinyRobot1"}',
        stream: 'stdout',
      },
      {
        log:
          'INFO:app.BookKeeper.task_summary:{"fleet_name": "tinyRobot", "task_id": "Clean2", "task_profile": {"task_id": "Clean2", "submission_time": {"sec": 131, "nanosec": 552120070}, "description": {"start_time": {"sec": 145383402, "nanosec": 0}, "priority": {"value": 0}, "task_type": {"type": 4}, "station": {"task_id": "", "robot_type": "", "place_name": ""}, "loop": {"task_id": "", "robot_type": "", "num_loops": "", "start_name": "", "finish_name": ""}, "delivery": {"task_id": "", "items": [], "pickup_place_name": "", "pickup_dispenser": "", "pickup_behavior": {"name": "", "parameters": []}, "dropoff_place_name": "", "dropoff_ingestor": "", "dropoff_behavior": {"name": "", "parameters": []}}, "clean": {"start_waypoint": "cleanzone"}}}, "state": 0, "status": "test_status3", "submission_time": {"sec": 0, "nanosec": 0}, "start_time": {"sec": 1623283162, "nanosec": 348332939}, "end_time": {"sec": 162593449, "nanosec": 79154833}, "robot_name": "tinyRobot3"}',
        stream: 'stdout',
      },
    ];

    const options = {
      method: 'POST',
      body: JSON.stringify(rmfServerData),
      headers: {
        'Content-Type': 'application/json',
        credentials: 'include',
        'Allow-Cross-Origin-Headers': 'true',
      },
    };

    try {
      const res = await fetch(`http://localhost:8003/log/rmfserver`, options);
      return await res.text();
    } catch (error) {
      console.log(error);
    }
  });

  it('header bar is present', async () => {
    const title = $('h6=Reports');
    expect(title).toBeVisible();
  });

  it('retrieve task report', async () => {
    const taskButton = $('.MuiList-root .MuiListItem-root:nth-child(8)').$('div*=Tasks');
    taskButton.click();
    const from = $('#fromLogDate-datetime-local');
    const to = $('#toLogDate-datetime-local');
    from.clearValue();
    from.setValue('06/01/2021 16:43');
    to.clearValue();
    to.setValue('06/22/2021 23:59');
    $('button=Retrieve Logs').click();
    expect($('h6=Task Summary')).toBeVisible();
    expect(browser.react$('TaskSummaryReportTable')).toBeVisible();
  });
});
