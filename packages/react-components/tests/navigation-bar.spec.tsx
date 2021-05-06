import React from 'react';
import { render, cleanup, screen } from '@testing-library/react';
import { NavigationBar } from '../lib/navigation-bar';
import { Tab } from '@material-ui/core';

describe('Banner Tab', () => {
  it('renders correctly', () => {
    const mockOnTabChange = jasmine.createSpy();
    render(
      <NavigationBar onTabChange={mockOnTabChange} value={'building'}>
        <Tab
          key={'building-tab'}
          label={'Building'}
          value={'building'}
          aria-label={`building-tab`}
        />
        <Tab key={'robots'} label={'Robots'} value={'robots'} aria-label={`building-tab`} />
      </NavigationBar>,
    );
    expect(screen.getByText('Building')).toBeTruthy();
    cleanup();
  });
});
