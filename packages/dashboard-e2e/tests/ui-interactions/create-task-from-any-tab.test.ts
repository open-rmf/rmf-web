import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit task from System Overview tab', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="System Overview"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('#task-type')).click();
    const getDeliveryOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Delivery - 1:1') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getDeliveryOption()));
    const deliveryOption = (await getDeliveryOption())!;
    await deliveryOption.click();

    await (await $('#pickup-location')).click();
    const getPantryOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'pantry') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getPantryOption()));
    const pantryOption = (await getPantryOption())!;
    await pantryOption.click();

    await (await $('#dropoff-location')).click();
    const getCoeOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'coe') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getCoeOption()));
    const coeOption = (await getCoeOption())!;
    await coeOption.click();

    await (await $('button[aria-label="Submit Now"]')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  }).timeout(60000);
});
