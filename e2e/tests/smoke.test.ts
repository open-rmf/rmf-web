describe('smoke test', () => {
  it('can load', () => {
    browser.url('/');
    const viz = $('#schedule-visualizer');
    viz.waitForDisplayed();
    expect(viz).toBeVisible();
  });
});
