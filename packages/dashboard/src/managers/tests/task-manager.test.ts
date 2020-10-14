import TaskManager from '../task-manager';

test('Status is been formatted correctly', () => {
  const rawStatus = 'Finding a plan for [tinyRobot/tinyRobot1] to go to [23] | Remaining phases: 6';
  // "Moving [tinyRobot/tinyRobot1]: ( 10.2479 -3.09206  1.16262) -> ( 6.26403 -3.51569  1.16864) | Remaining phases: 1 | Remaining phases: 6"
  const status = TaskManager.formatStatus(rawStatus);
  // expect(status)
});
