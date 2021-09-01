import { getDoorAccordion, getOmniPanel, omniPanelMainMenu } from '../utils';

describe('door controls', () => {
  beforeEach(omniPanelMainMenu);

  it('clicking on open button opens the door', () => {
    const omnipanel = getOmniPanel();
    omnipanel.$('h5=Doors').click();
    const elem = getDoorAccordion('main_door');
    elem.click(); // open accordion
    const btn = $('button=Open');
    btn.click();
    expect(elem.$('[role=status]')).toHaveText('OPEN', { wait: 30000 });
  });

  it('clicking on close button closes the door', () => {
    const omnipanel = getOmniPanel();
    omnipanel.$('h5=Doors').click();
    const elem = getDoorAccordion('main_door');
    elem.click(); // open accordion
    const btn = $('button=Close');
    btn.click();
    expect(elem.$('[role=status]')).toHaveText('CLOSED', { wait: 30000 });
  });
});
