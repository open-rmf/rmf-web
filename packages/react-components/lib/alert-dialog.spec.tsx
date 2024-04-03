import { createTheme, ThemeProvider } from '@mui/material';
import { fireEvent, render } from '@testing-library/react';
import React from 'react';
import { AlertContent, AlertDialog } from './alert-dialog';

const theme = createTheme();

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
    const acknowledge = jest.fn();
    const dismiss = jest.fn();
    const root = render(
      <ThemeProvider theme={theme}>
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
    fireEvent.click(root.getByText('Dismiss'));
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
    const acknowledge = jest.fn();
    const close = jest.fn();
    const root = render(
      <ThemeProvider theme={theme}>
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
    fireEvent.click(root.getByText('Acknowledge'));
    expect(acknowledge).toHaveBeenCalled();
    // acknowledge button turns to acknowledged
    expect(() => root.getByText('Acknowledge')).toThrow();
    expect(() => root.getByText('Acknowledged')).not.toThrow();
    // dismiss button turns to close
    expect(() => root.getByText('Dismiss')).toThrow();
    expect(() => root.getByText('Close')).not.toThrow();
    fireEvent.click(root.getByText('Close'));
    expect(close).toHaveBeenCalled();
  });
});
