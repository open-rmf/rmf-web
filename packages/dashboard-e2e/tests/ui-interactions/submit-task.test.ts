import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit loop task', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Tasks"]')).click();
    await (await $('button[aria-label="Create Task"]')).click();
    await (await $('#task-type')).click();
    const getLoopOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Loop') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getLoopOption()));
    const loopOption = (await getLoopOption())!;
    await loopOption.click();

    await (await $('#place-input')).setValue('coe');

    await (await $('button=Submit')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  }).timeout(60000);
});
