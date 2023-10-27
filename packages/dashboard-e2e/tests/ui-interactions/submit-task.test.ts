import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit patrol task', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Tasks"]')).click();
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

  it('can submit clean task', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Tasks"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('#task-type')).click();
    const getCleanOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Clean') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getCleanOption()));
    const cleanOption = (await getCleanOption())!;
    await cleanOption.click();

    await (await $('#cleaning-zone')).click();
    const getRestaurantOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'clean_restaurant') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getRestaurantOption()));
    const restaurantOption = (await getRestaurantOption())!;
    await restaurantOption.click();

    await (await $('button[aria-label="Submit Now"]')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  }).timeout(60000);

  it('can submit favorite task', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Tasks"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('#task-type')).click();
    const getCleanOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Clean') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getCleanOption()));
    const cleanOption = (await getCleanOption())!;
    await cleanOption.click();

    await (await $('#cleaning-zone')).click();
    const getRestaurantOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'clean_restaurant') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getRestaurantOption()));
    const restaurantOption = (await getRestaurantOption())!;
    await restaurantOption.click();

    await (await $('button[aria-label="Save as a favorite task"]')).click();
    await (await $('#favorite-input')).waitForDisplayed();
    await (await $('#favorite-input')).setValue('My favorite task');
    await (await $('button[aria-label="Confirm"]')).click();

    await expect($('div=Created favorite task successfully')).toBeDisplayed();

    await (await $('button[aria-label="Cancel button"]')).click();
  }).timeout(60000);
});
