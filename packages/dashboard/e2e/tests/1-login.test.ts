import { login } from './utils';

describe('login', () => {
  it('can login', () => {
    login();
    console.log('Browser url ----->>>>>>: ' + new URL(browser.getUrl()).pathname);
    $$('div').map((val) => {
      console.log(val.getHTML());
    });
    expect(browser.waitUntil(() => new URL(browser.getUrl()).pathname === '/')).toBeTruthy();
  });

  it('subsequent visits do not require login', () => {
    browser.url('/');
    expect($('#appbar')).toBeVisible();
  });
});
