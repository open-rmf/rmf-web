import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit task from System Overview tab', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="System Overview"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('#task-type')).click();
    const getPatrolOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Patrol') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getPatrolOption()));
    const patrolOption = (await getPatrolOption())!;
    await patrolOption.click();
    await (await $('#place-input')).click();

    const getShopOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'shop') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getShopOption()));
    const shopOption = (await getShopOption())!;
    await shopOption.click();
    await (await $('button[aria-label="Submit Now"]')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  }).timeout(60000);
});
