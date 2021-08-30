import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit loop task', () => {
    const appBar = getAppBar();
    appBar.$('span=Tasks').click();
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
  }).timeout(20000);
});
