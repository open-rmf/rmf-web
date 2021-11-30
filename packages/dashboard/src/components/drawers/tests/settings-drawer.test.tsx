import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { Settings } from '../../../settings';
import { SettingsContext } from '../../app-contexts';
import SettingsDrawer from '../../drawers/settings-drawer';

describe('Settings Drawer', () => {
  let settings: Settings;
  let onSettingsChange: jest.Mock;
  let handleCloseButton: jest.Mock;

  beforeEach(() => {
    settings = { themeMode: 0 };
    onSettingsChange = jest.fn();
    handleCloseButton = jest.fn();
  });

  it('should render correctly', () => {
    const root = render(
      <SettingsContext.Provider value={settings}>
        <SettingsDrawer
          handleCloseButton={handleCloseButton}
          settings={settings}
          onSettingsChange={onSettingsChange}
          open={true}
        />
      </SettingsContext.Provider>,
    );
    root.unmount();
  });

  it('should call handleCloseButton function when closeIcon button is clicked', () => {
    const root = render(
      <SettingsContext.Provider value={settings}>
        <SettingsDrawer
          settings={settings}
          onSettingsChange={(newSettings) => onSettingsChange(newSettings)}
          open={true}
          handleCloseButton={handleCloseButton}
        />
      </SettingsContext.Provider>,
    );
    const closeButton = root.getByRole('button');
    userEvent.click(closeButton);
    expect(handleCloseButton).toHaveBeenCalledTimes(1);
  });
});
