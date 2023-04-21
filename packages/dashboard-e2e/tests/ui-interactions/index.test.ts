// we could just import the test cases as per normal import,
// but doing it this way allows us to correctly populate the wdio test suite name.
describe('ui interactions', async () => {
  await import('./door-controls.test');
  await import('./submit-task.test');
  await import('./create-task-from-any-tab.test');
});
