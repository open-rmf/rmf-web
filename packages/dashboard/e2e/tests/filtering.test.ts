import { Element } from '@wdio/sync';
import { makeLauncher } from '../../rmf-launcher';
import { login, overwriteClick } from './utils';

describe('Simple Filter', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  before(() => {
    $('[data-component=MainMenu] [data-item=Doors]').click();
    console.log($('[data-component=simple-filter]').getHTML());
  });

  it('testing', () => {
    console.log('blah');
    // set value to filter input
    $('[data-component=simple-filter]').$('input').setValue('value');
    console.log($('[data-component=simple-filter]').$('input').getValue());
    const door = $('[data-component=DoorMarker]');
    door.waitForClickable();
    door.click();
    // $('data-component=DoorMarker').click();
    expect($('[data-component=simple-filter]').$('input').getValue()).toEqual('');
  });
});
