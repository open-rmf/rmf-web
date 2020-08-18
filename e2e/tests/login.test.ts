import { RmfLauncher } from '../rmf-launcher';
import { login } from './utils';

describe('login', () => {
  const launcher = new RmfLauncher();

  before(async () => await launcher.launch());
  after(async () => await launcher.kill());

  it('can login', () => {
    login();
    expect(browser.waitUntil(() => new URL(browser.getUrl()).pathname === '/')).toBeTruthy();
  });

  it('subsequent visits do not require login', () => {
    browser.url('/');
    expect($('#ScheduleVisualizer')).toBeVisible();
  });
});
