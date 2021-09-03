import { getDoorAccordion, getOmniPanel, omniPanelMainMenu } from '../utils';

describe('door controls', () => {
  beforeEach(omniPanelMainMenu);

  it('clicking on open button opens the door', async () => {
    const omnipanel = await getOmniPanel();
    (await omnipanel.$('h5=Doors')).click();
    const elem = await getDoorAccordion('main_door');
    await elem.click(); // open accordion
    const btn = await $('button=Open');
    await btn.click();
    await expect(elem.$('[role=status]')).toHaveText('OPEN', { wait: 30000 });
  });

  it('clicking on close button closes the door', async () => {
    const omnipanel = await getOmniPanel();
    (await omnipanel.$('h5=Doors')).click();
    const elem = await getDoorAccordion('main_door');
    await elem.click(); // open accordion
    const btn = await $('button=Close');
    await btn.click();
    await expect(elem.$('[role=status]')).toHaveText('CLOSED', { wait: 30000 });
  });
});
