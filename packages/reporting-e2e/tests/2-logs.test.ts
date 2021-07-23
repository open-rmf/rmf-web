import { login, overwriteClick } from './utils';
import { rmfData } from './mock-data';
import fetch from 'node-fetch';

describe('logs', () => {
  before(() => overwriteClick());
  before(() => browser.url('/'));

  before(login);

  it('should store the logs correctly', async () => {
    const options = {
      method: 'POST',
      body: JSON.stringify(rmfData),
      headers: {
        'Content-Type': 'application/json',
      },
    };

    let response;
    try {
      const res = await fetch(`http://localhost:8003/log/rmfserver`, options);
      response = await res.text();
    } catch (error) {
      console.log(error);
    }

    expect(response).toBe('"Logs were saved correctly"');
  });
});
