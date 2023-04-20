import { ThemeProvider } from '@mui/material';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { AlertContent, AlertDialog } from './alert-dialog';
import defaultTheme from '@mui/material/styles/defaultTheme';

describe('AcknowledgeAndCloseAlertDialog', () => {
  it('dismiss button called', () => {
    const buildAlertDialogContent = (): AlertContent[] => {
      return [
        {
          title: 'ID',
          value: 'testAlertID',
        },
        {
          title: 'Error logs',
          value: '1/1/1970 00:00:00 - error',
        },
        {
          title: 'Logs',
          value: '1/1/1970 00:00:00 - completed',
        },
      ];
    };
    const acknowledge = jasmine.createSpy();
    const dismiss = jasmine.createSpy();
    const root = render(
      <ThemeProvider theme={defaultTheme}>
        <AlertDialog
          key={'testAlert'}
          onDismiss={dismiss}
          onAcknowledge={acknowledge}
          title={'alertTitle'}
          progress={1}
          alertContents={buildAlertDialogContent()}
          backgroundColor={'ffff'}
        />
      </ThemeProvider>,
    );
    expect(() => root.getByText('Acknowledge')).not.toThrow();
    expect(() => root.getByText('Dismiss')).not.toThrow();
    userEvent.click(root.getByText('Dismiss'));
    expect(dismiss).toHaveBeenCalled();
  });

  it('acknowledge and close', () => {
    const buildAlertDialogContent = (): AlertContent[] => {
      return [
        {
          title: 'ID',
          value: 'testAlertID',
        },
        {
          title: 'Error logs',
          value: '1/1/1970 00:00:00 - error',
        },
        {
          title: 'Logs',
          value: '1/1/1970 00:00:00 - completed',
        },
      ];
    };
    const acknowledge = jasmine.createSpy();
    const close = jasmine.createSpy();
    const root = render(
      <ThemeProvider theme={defaultTheme}>
        <AlertDialog
          key={'testAlert'}
          onDismiss={close}
          onAcknowledge={acknowledge}
          title={'alertTitle'}
          progress={1}
          alertContents={buildAlertDialogContent()}
          backgroundColor={'ffff'}
        />
      </ThemeProvider>,
    );
    expect(() => root.getByText('Acknowledge')).not.toThrow();
    expect(() => root.getByText('Dismiss')).not.toThrow();
    userEvent.click(root.getByText('Acknowledge'));
    expect(acknowledge).toHaveBeenCalled();
    // acknowledge button turns to acknowledged
    expect(() => root.getByText('Acknowledge')).toThrow();
    expect(() => root.getByText('Acknowledged')).not.toThrow();
    // dismiss button turns to close
    expect(() => root.getByText('Dismiss')).toThrow();
    expect(() => root.getByText('Close')).not.toThrow();
    userEvent.click(root.getByText('Close'));
    expect(close).toHaveBeenCalled();
  });
});
