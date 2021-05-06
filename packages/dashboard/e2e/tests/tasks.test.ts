import { makeLauncher } from '../../rmf-launcher';
import { login } from './utils';

describe('tasks', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());
  before(login);

  it('can submit loop task', () => {
    browser.url('/tasks');
    $('button[aria-label="Create Task"]').click();
    $('#task-type').click();
    const getLoopOption = () => $$('[role=option]').find((elem) => elem.getText() === 'Loop');
    browser.waitUntil(() => !!getLoopOption());
    const loopOption = getLoopOption()!;
    loopOption.click();

    $('#start-location').setValue('coe');
    $('#finish-location').setValue('pantry');

    $('button[aria-label="Submit"]').click();
    $('div=Successfully created task').waitForDisplayed();
  });
});
