import { Element } from '@wdio/sync';
import { makeLauncher } from '../../rmf-launcher';
import { login, overwriteClick } from './utils';

describe('door request', () => {
  const launcher = makeLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  before(() => {
    $('[data-component=MainMenu] [data-item=Doors]').click();
    console.log($('[data-item=door-filter]').getHTML());
    console.log($('.MuiAccordion-root*=main_door').getHTML());
    // doorAccordion = $('.MuiAccordion-root*=main_door');
    // doorAccordion.click();
  });

  it('testing', () => {
    console.log('blah');
  });
});
