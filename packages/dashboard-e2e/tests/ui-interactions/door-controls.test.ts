import { getDoorCell } from '../utils';

describe('door controls', () => {
  it('clicking on open button opens the door', async () => {
    const doorCell = await getDoorCell('main_door');
    const openBtn = await doorCell.$('button=Open');
    await openBtn.click();
    await expect(doorCell.$('[role=status]')).toHaveText('OPEN', { wait: 30000 });
  });

  it('clicking on close button closes the door', async () => {
    const doorCell = await getDoorCell('main_door');
    const openBtn = await doorCell.$('button=Close');
    await openBtn.click();
    await expect(doorCell.$('[role=status]')).toHaveText('CLOSED', { wait: 30000 });
  });
});
