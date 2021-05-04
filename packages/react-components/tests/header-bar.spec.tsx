import React from 'react';
import { render, cleanup, screen } from '@testing-library/react';
import { HeaderBar } from '../lib/header-bar';
import { NavigationBar } from '../lib/navigation-bar';
import Tab from '@material-ui/core/Tab';

describe('Header Bar', () => {
  it('renders correctly', () => {
    const mockOnTabChange = jasmine.createSpy();
    render(
      <HeaderBar>
        <NavigationBar onTabChange={mockOnTabChange} value={'building'}>
          <Tab
            key={'building-tab'}
            label={'Building'}
            value={'building'}
            aria-label={`building-tab`}
          />
          <Tab key={'robots-tab'} label={'Robots'} value={'robots'} aria-label={`building-tab`} />
        </NavigationBar>
      </HeaderBar>,
    );
    expect(screen.getByText('Building')).toBeTruthy();
    expect(screen.getByText('Robots')).toBeTruthy();
    cleanup();
  });
});
