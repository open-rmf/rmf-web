import { createMuiTheme } from '@material-ui/core';
import { Alerts } from '../alerts';

const theme = createMuiTheme();

describe('Verification Alert', () => {
  test('Correct generation of default params values', () => {
    const verification = Alerts.getVerificationOptions({});
    expect(verification).toMatchSnapshot();
  });

  test('Correct generation of custom params values', () => {
    const title = 'test';
    const text = 'test';
    const icon = 'error';
    const confirmButtonText = 'test';
    const cancelButtonText = 'test';

    const verification = Alerts.getVerificationOptions({
      title: title,
      body: text,
      icon: icon,
      confirmButtonText: confirmButtonText,
      cancelButtonText: cancelButtonText,
    });
    expect(verification).toMatchSnapshot();
  });
});

describe('Success Alert', () => {
  test('Correct generation of default params values', () => {
    const success = Alerts.getSuccessOptions();
    expect(success).toEqual({
      title: 'Done!',
      text: 'Successful Operation',
      icon: 'success',
      timer: 2000,
      heightAuto: false,
    });
  });

  test('Correct generation of custom params values', () => {
    const success = Alerts.getSuccessOptions('test');
    expect(success.text).toBe('test');
  });
});

describe('Error Alert', () => {
  test('Correct generation of default params values', () => {
    const error = Alerts.getErrorOptions();
    expect(error).toEqual({
      title: 'Ups',
      text: 'An error has occurred',
      icon: 'error',
      heightAuto: false,
    });
  });

  test('Correct generation of custom params values', () => {
    const error = Alerts.getErrorOptions('test');
    expect(error.text).toBe('test');
  });
});
