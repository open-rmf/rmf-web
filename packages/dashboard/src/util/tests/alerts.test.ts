import { createMuiTheme } from '@material-ui/core';
import { Alerts } from '../alerts';

const theme = createMuiTheme();

describe('Verification Alert', () => {
  test('Correct generation of default params values', () => {
    const verification = Alerts.getVerificationOptions({});
    expect(verification).toMatchSnapshot();
  });

  test('Correct generation of custom params values', () => {
    const verification = Alerts.getVerificationOptions({
      title: 'test',
      body: 'test',
      icon: 'error',
      confirmButtonText: 'test',
      cancelButtonText: 'test',
    });
    expect(verification).toMatchSnapshot();
  });
});

describe('Success Alert', () => {
  test('Correct generation of default params values', () => {
    const success = Alerts.getSuccessOptions();
    expect(success).toMatchSnapshot();
  });

  test('Correct generation of custom params values', () => {
    const success = Alerts.getSuccessOptions('test');
    expect(success.text).toBe('test');
  });
});

describe('Error Alert', () => {
  test('Correct generation of default params values', () => {
    const error = Alerts.getErrorOptions();
    expect(error).toMatchSnapshot();
  });

  test('Correct generation of custom params values', () => {
    const error = Alerts.getErrorOptions('test');
    expect(error.text).toBe('test');
  });
});
