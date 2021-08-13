import { Element } from '@wdio/sync';
import { makeLauncher } from '../../dashboard/rmf-launcher';
import { login, overwriteClick } from './utils';

describe('ui interactions', () => {
  let doorCell: Element;
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  // door interaction test
  it('clicking on open button opens the door', () => {
    doorCell = $('[data-item=main_door]');
    $('button[aria-label="main_door_open"]').click();
    expect(doorCell.$('p=OPEN')).toBeDisplayed();
  });

  it('clicking on close button closes the door', () => {
    doorCell = $('[data-item=main_door]');
    $('button[aria-label="main_door_close"]').click();
    expect(doorCell.$('p=CLOSED')).toBeDisplayed();
  });

  // submit task test
  it('can submit loop task', () => {
    browser.url('/tasks');
    $('button[aria-label="Create Task"]').click();
    $('#task-type').click();
    const getLoopOption = () => $$('[role=option]').find((elem) => elem.getText() === 'Loop');
    browser.waitUntil(() => !!getLoopOption());
    const loopOption = getLoopOption()!;
    loopOption.click();

    $('#start-location').setValue('coe');
    $('#finish-location').setValue('pantry');

    $('button[aria-label="Submit"]').click();
    browser.waitUntil(() => $('div=Successfully created task').isDisplayed(), {
      timeout: 15000,
      interval: 500,
    });
  }).timeout(20000);
});
