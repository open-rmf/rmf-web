import { login } from './utils';

describe('login', () => {
  it('can login', () => {
    console.log('<<<<<<<<<<<<<<<< I can login >>>>>>>>>>>>>>>>>>>');
    login();
    expect(browser.waitUntil(() => new URL(browser.getUrl()).pathname === '/')).toBeTruthy();
  });

  it('subsequent visits do not require login', () => {
    console.log('<<<<<<<<<<<<<<<< No login again!!!! >>>>>>>>>>>>>>>>>>>');
    browser.url('/');
    expect($('#appbar')).toBeVisible();
  });
});
