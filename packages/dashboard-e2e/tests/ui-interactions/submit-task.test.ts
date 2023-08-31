import { getAppBar } from '../utils';

describe('submit task', () => {
  it('can submit patrol task', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Tasks"]')).click();
    await (await appBar.$('button[aria-label="new task"]')).click();
    await (await $('#task-type')).click();
    const getDeliveryOption = async () => {
      const options = await $$('[role=option]');
      for (const opt of options) {
        const text = await opt.getText();
        if (text === 'Delivery') {
          return opt;
        }
      }
      return null;
    };
    await browser.waitUntil(async () => !!(await getDeliveryOption()));
    const deliveryOption = (await getDeliveryOption())!;
    await deliveryOption.click();

    await (await $('#pickup-location')).setValue('pantry');
    await (await $('#dropoff-location')).setValue('coe');

    await (await $('button[aria-label="Submit Now"]')).click();
    await expect($('div=Successfully created task')).toBeDisplayed();
  }).timeout(60000);
});
