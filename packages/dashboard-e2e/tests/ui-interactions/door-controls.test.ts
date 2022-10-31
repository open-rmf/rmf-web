import { getAppBar, getDoorCell } from '../utils';

describe('door controls', () => {
  it('clicking on open button opens the door', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="Infrastructure"]')).click();
    await (await $('button[aria-label="open-main_door"]')).click();
    await expect($('[role=status]')).toHaveText('OPEN', { wait: 30000 });
  });

  // it('clicking on close button closes the door', async () => {
  //   const appBar = await getAppBar();
  //   await (await appBar.$('button[aria-label="Infrastructure"]')).click();
  //   const doorCell = await $('[aria-labelledby="door-cell-main_door"]');
  //   const closeBtn = await doorCell.$('button=Close');
  //   await closeBtn.click();
  //   await expect(doorCell.$('[role=status]')).toHaveText('CLOSED', { wait: 30000 });
  // });
});
