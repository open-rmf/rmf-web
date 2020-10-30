import TaskManager from '../task-manager';

test('Get name of the robot from status', () => {
  const rawStatus = 'Finding a plan for [tinyRobot/tinyRobot1] to go to [23] | Remaining phases: 6';
  const actor = TaskManager.getActorFromStatus(rawStatus);
  expect(actor).toEqual(['[tinyRobot/tinyRobot1]']);
});
