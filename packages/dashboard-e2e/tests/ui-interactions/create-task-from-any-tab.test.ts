import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit task from System Overview tab', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="System Overview"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('button[aria-label="Submit Now"]')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  });
});
