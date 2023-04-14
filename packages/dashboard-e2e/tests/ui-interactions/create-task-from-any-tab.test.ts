import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit task from robot tab', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Robots"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('button=Submit')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  });
});
