import fetch from 'node-fetch';
import { rmfData } from './mock-data';
import { getReport, overwriteClick } from './utils';

describe('reporting interactions', () => {
  before(() => overwriteClick());
  before(() => browser.url('/'));

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
      await res.text();
    } catch (error) {
      console.log(error);
    }
  });

  it('should retrieve dispenser state report', async () => {
    const options = {
      listOrder: 2,
      elemName: 'div*=Dispensers',
      reportTitle: 'h6*=Dispenser State',
    };
    await getReport(options);
    await expect($('h6*=Dispenser State')).toBeDisplayed();
  });

  it('should retrieve door state report', async () => {
    const options = {
      listOrder: 3,
      elemName: 'div*=Doors',
      reportTitle: 'h6*=Door State',
    };
    await getReport(options);
    await expect($('h6*=Door State')).toBeDisplayed();
  });

  it('should retrieve the fleet state report', async () => {
    const options = {
      listOrder: 4,
      elemName: 'div*=Fleets',
      reportTitle: 'h6*=Fleet State',
    };
    await getReport(options);
    await expect($('h6*=Fleet State')).toBeDisplayed();
  });

  it('should retrieve the health report', async () => {
    const options = {
      listOrder: 5,
      elemName: 'div*=Health',
      reportTitle: 'h6*=Health',
    };
    await getReport(options);
    await expect($('h6*=Health')).toBeDisplayed();
  });

  it('should retrieve ingestor state report', async () => {
    const options = {
      listOrder: 6,
      elemName: 'div*=Ingestor',
      reportTitle: 'h6*=Ingestor State',
    };
    await getReport(options);
    await expect($('h6*=Ingestor State')).toBeDisplayed();
  });

  it('should retrieve lift state report', async () => {
    const options = {
      listOrder: 7,
      elemName: 'div*=Lifts',
      reportTitle: 'h6*=Lift State',
    };
    await getReport(options);
    await expect($('h6*=Lift State')).toBeDisplayed();
  });

  it('should retrieve the task summary report', async () => {
    const options = {
      listOrder: 8,
      elemName: 'div*=Tasks',
      reportTitle: 'h6*=Task',
    };
    await getReport(options);
    await expect($('h6*=Task')).toBeDisplayed();
  });
});
