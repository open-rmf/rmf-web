import { getAppBar } from '../utils';

describe('door controls', () => {
  it('door starts closed', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Infrastructure"]')).click();
    const doorCell = await $('[aria-labelledby="door-cell-coe_door"]');
    await expect(doorCell.$('[role=status]')).toHaveText('CLOSED', { wait: 180000 });
  });

  it('clicking on open button opens the door', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Infrastructure"]')).click();
    const doorCell = await $('[aria-labelledby="door-cell-coe_door"]');
    const openBtn = await doorCell.$('button[aria-label="open-coe_door"]');
    await openBtn.click();
    await expect(doorCell.$('[role=status]')).toHaveText('OPEN', { wait: 180000 });
  });

  it('clicking on close button closes the door', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Infrastructure"]')).click();
    const doorCell = await $('[aria-labelledby="door-cell-coe_door"]');
    const closeBtn = await doorCell.$('button[aria-label="close-coe_door"]');
    await closeBtn.click();
    await expect(doorCell.$('[role=status]')).toHaveText('CLOSED', { wait: 180000 });
  });
});
