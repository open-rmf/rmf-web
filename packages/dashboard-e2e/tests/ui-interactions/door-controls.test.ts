import { getAppBar } from '../utils';

describe('door controls', () => {
  it('door starts closed', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="System Overview"]')).click();
    const doorState = $('.MuiTableCell-alignLeft:nth-child(4) p');
    expect(doorState.getText('CLOSED'));
  });

  it('clicking on open button opens the door', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="System Overview"]')).click();
    const openButton = $('button[aria-label="open"]');
    openButton.click();
    const doorState = $('.MuiTableCell-alignLeft:nth-child(4) p');
    expect(doorState.getText('OPEN'));
  });

  it('clicking on close button closes the door', async () => {
    const appBar = await getAppBar();
    await (await appBar.$('button[aria-label="System Overview"]')).click();
    const openButton = $('button[aria-label="close"]');
    openButton.click();
    //test another way to get cell
    const doorState = await $('table tr td:nth-child(4) [data-testid="door-state"]');
    expect(doorState.getText('CLOSED'));
  });
});
