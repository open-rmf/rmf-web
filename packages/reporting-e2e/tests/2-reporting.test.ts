import { login, overwriteClick, selectDateAndRetrieveLogs } from './utils';
import { rmfData } from './mock-data';
import fetch from 'node-fetch';

describe('reporting interactions', () => {
  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);
  before(async () => {
    const options = {
      method: 'POST',
      body: JSON.stringify(rmfData),
      headers: {
        'Content-Type': 'application/json',
      },
    };

    try {
      const res = await fetch(`http://localhost:8003/log/rmfserver`, options);
      return await res.json();
    } catch (error) {
      console.log(error);
    }
  });

  it('should retrieve dispenser state report', async () => {
    browser.waitUntil(() => $('.MuiList-root').waitForDisplayed() === true);
    const dispenserButton = $('.MuiList-root .MuiListItem-root:nth-child(2)').$('div*=Dispensers');
    dispenserButton.click();
    selectDateAndRetrieveLogs();
    browser.waitUntil(() => $('h6=Dispenser State').waitForDisplayed({ timeout: 5000 }) === true);
    expect($('h6=Dispenser State')).toBeVisible();
  });
  it('should retrieve door state report', async () => {
    browser.waitUntil(() => $('.MuiList-root').waitForDisplayed() === true);
    const dispenserButton = $('.MuiList-root .MuiListItem-root:nth-child(3)').$('div*=Doors');
    dispenserButton.click();
    selectDateAndRetrieveLogs();
    browser.waitUntil(() => $('h6=Door State').waitForDisplayed({ timeout: 5000 }) === true);
    expect($('h6=Door State')).toBeVisible();
  });

  it('should retrieve the fleet state report', async () => {
    browser.waitUntil(() => $('.MuiList-root').waitForDisplayed() === true);
    const fleetsButton = $('.MuiList-root .MuiListItem-root:nth-child(4)').$('div*=Fleets');
    fleetsButton.click();
    selectDateAndRetrieveLogs();
    browser.waitUntil(() => $('h6=Fleet State').waitForDisplayed({ timeout: 10000 }) === true);
    expect($('h6=Fleet State')).toBeVisible();
  });

  it('should health report', async () => {
    browser.waitUntil(() => $('.MuiList-root').waitForDisplayed() === true);
    const healthButton = $('.MuiList-root .MuiListItem-root:nth-child(5)').$('div*=Health');
    healthButton.click();
    selectDateAndRetrieveLogs();
    browser.waitUntil(() => $('h6=Health').waitForDisplayed({ timeout: 5000 }) === true);
    expect($('h6=Health')).toBeVisible();
  });

  it('should retrieve ingestor state report', async () => {
    browser.waitUntil(() => $('.MuiList-root').waitForDisplayed() === true);
    const ingestorButton = $('.MuiList-root .MuiListItem-root:nth-child(6)').$('div*=Ingestor');
    ingestorButton.click();
    selectDateAndRetrieveLogs();
    browser.waitUntil(() => $('h6=Ingestor State').waitForDisplayed({ timeout: 5000 }) === true);
    expect($('h6=Ingestor State')).toBeVisible();
  });

  it('should retrieve lift state report', async () => {
    browser.waitUntil(() => $('.MuiList-root').waitForDisplayed() === true);
    const liftButton = $('.MuiList-root .MuiListItem-root:nth-child(7)').$('div*=Lifts');
    liftButton.click();
    selectDateAndRetrieveLogs();
    browser.waitUntil(() => $('h6=Lift State').waitForDisplayed({ timeout: 5000 }) === true);
    expect($('h6=Lift State')).toBeVisible();
  });

  it('should retrieve the task summary report', async () => {
    browser.waitUntil(() => $('.MuiList-root').waitForDisplayed() === true);
    const taskButton = $('.MuiList-root .MuiListItem-root:nth-child(8)').$('div*=Tasks');
    taskButton.click();
    selectDateAndRetrieveLogs();
    browser.waitUntil(() => $('h6=Task Summary').waitForDisplayed({ timeout: 10000 }) === true);
    expect($('h6=Task Summary')).toBeVisible();
  });
});
