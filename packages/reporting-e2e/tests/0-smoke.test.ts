describe('smoke test', () => {
  it('can load', () => {
    browser.url('/');
    expect($('#appbar')).toBeVisible();
  });
});
