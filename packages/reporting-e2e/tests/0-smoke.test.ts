describe('smoke test', () => {
  it('can load', () => {
    browser.url('/');
    expect($('h4*=Reporting')).toBeDisplayed();
  });
});
